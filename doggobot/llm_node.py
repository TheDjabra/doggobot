#!/usr/bin/env python3
"""The slow path: loose speech -> a structured command, via a local LLM.

Subscribes /voice_unparsed (what the keyword parser could not handle) and
publishes a structured command to /voice_cmd, the same topic buttons and matched
speech already use. Nothing downstream knows a model was involved.

**Self-hosted by choice.** The model runs on a machine you own, reached over
Tailscale, rather than a vendor API. Everything else in this project already runs
on hardware under your control (detector on the camera's VPU, speech offline on
the Pi, tracking on-camera), and the parser matching that is both a better
architecture story and one less external dependency.

**The schema is the safety mechanism.** Ollama constrains decoding to a JSON
schema, so the model physically cannot emit a primitive that does not exist. This
is the same trick as the Vosk grammar on the microphone: rather than trusting a
model to behave, make misbehaviour unrepresentable. A 7B model is plenty for
mapping "spin left a couple of times" onto an enum that already exists.

**It can only ever add.** If the model is unreachable, slow, or returns
nonsense, the utterance is dropped and the car behaves exactly as it does with no
LLM at all. The keyword path covers every graded requirement on its own, so this
tier is strictly additive and must never become load-bearing.
"""
import json
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ACTIONS = ['forward', 'reverse', 'circle_left', 'circle_right',
           'turn_around', 'three_point', 'figure_eight', 'wait', 'stop',
           'follow']
COLORS = ['green', 'red']

# Constrains decoding, so the model cannot invent an action or a colour.
SCHEMA = {
    'type': 'object',
    'properties': {
        'understood': {'type': 'boolean'},
        'steps': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {
                    'action': {'type': 'string', 'enum': ACTIONS},
                    'seconds': {'type': 'number'},
                    'metres': {'type': 'number'},
                    'degrees': {'type': 'number'},
                    'until_color': {'type': 'string', 'enum': COLORS},
                },
                'required': ['action'],
            },
        },
    },
    'required': ['understood', 'steps'],
}

SYSTEM = """You translate spoken commands for a small robot car into a list of steps.

WHAT THE CAR CAN PHYSICALLY DO. There is no "turn and keep going straight": the
only way this car changes direction is by driving in a circle.

  forward       drive straight ahead
  reverse       drive straight backwards
  circle_right  drive forward while turning right, continuously (an arc/circle)
  circle_left   drive forward while turning left, continuously (an arc/circle)
  turn_around   turn roughly 180 degrees and end up facing back the way it came
  three_point   a three-point turn: same result as turn_around but in much less
                space. Prefer it when the speaker mentions tight spaces or says
                "three point"
  figure_eight  one circle each way
  wait          hold still
  stop          stop and cancel everything
  follow        follow the nearest person

DIRECTION WORDS MEAN CIRCLES. "go right", "turn right", "head right", "spin
right", "veer right", "to the right" all mean circle_right. The same for left
and circle_left. Never translate a direction word into `forward` - forward means
straight ahead with no turn, and is only correct when no direction was given.

DURATIONS ATTACH TO THE STEP THEY WERE SAID WITH. "go right for 5 seconds then
left for 3 seconds" is two steps, the first with seconds=5 and the second with
seconds=3. Do not move a duration onto a different step and do not drop one.

Optional per step, at most ONE of:
  seconds   how long, in seconds
  metres    how far, in METRES - convert any other unit yourself:
            1 foot = 0.30, 6 inches = 0.15, half a foot = 0.15, 1 yard = 0.91
  degrees   how far to turn, in degrees, for circle_left / circle_right
And optionally `until_color` (drive until that colour is seen: green, red).

RULES
- Only use the actions listed. Never invent one.
- If the request cannot be expressed with those actions, set understood=false and
  return an empty steps list. Do NOT approximate: a wrong guess makes the car do
  something the person did not ask for.
- Repetitions become repeated steps ("twice" = the same step twice).
- Keep the order the person said things in.

COUNT BEFORE YOU ANSWER. The instructions are separated by "then" and "and".
Count how many the person gave and return exactly that many steps. Dropping the
LAST instruction is the most common mistake: check that the final thing they
said appears as the final step.

EXAMPLES
"go right for 5 seconds then left for 3 seconds"
-> {"understood": true, "steps": [{"action":"circle_right","seconds":5},
    {"action":"circle_left","seconds":3}]}

"go forward one metre then reverse half a foot"
-> {"understood": true, "steps": [{"action":"forward","metres":1},
    {"action":"reverse","metres":0.15}]}

"go right at a 30 degree angle"
-> {"understood": true, "steps": [{"action":"circle_right","degrees":30}]}

"drive forward two feet then turn around"
-> {"understood": true, "steps": [{"action":"forward","metres":0.61},
    {"action":"turn_around"}]}

"spin right two times then reverse"
-> {"understood": true, "steps": [{"action":"circle_right"},
    {"action":"circle_right"}, {"action":"reverse"}]}

"drive forward for five seconds then spin left twice"
-> {"understood": true, "steps": [{"action":"forward","seconds":5},
    {"action":"circle_left"}, {"action":"circle_left"}]}

"go until you see the green thing then stop"
-> {"understood": true, "steps": [{"action":"forward","until_color":"green"},
    {"action":"stop"}]}

"go forward then turn around and come back"
-> {"understood": true, "steps": [{"action":"forward"},
    {"action":"turn_around"}, {"action":"forward"}]}

"back up for two seconds then do a figure eight"
-> {"understood": true, "steps": [{"action":"reverse","seconds":2},
    {"action":"figure_eight"}]}

"make me a sandwich"
-> {"understood": false, "steps": []}"""


class LlmNode(Node):

    def __init__(self):
        super().__init__('llm_node')

        self.declare_parameter('host', 'http://rasputin:11434')
        self.declare_parameter('model', 'qwen2.5:7b-instruct')
        self.declare_parameter('timeout_s', 12.0)
        self.declare_parameter('max_steps', 8)
        # How long Ollama holds the model in VRAM after a request. Measured on
        # an RTX 3070: 50.9 s to load 4.7 GB cold, 0.34 s warm. The model is not
        # slow, it is unloaded, and Ollama's 5 minute default would evict it
        # between demo commands. "-1" means never evict.
        self.declare_parameter('keep_alive', '-1')
        # Fire a throwaway request at startup so the first real command never
        # pays the cold cost.
        self.declare_parameter('warm_on_start', True)

        g = self.get_parameter
        self.host = str(g('host').value).rstrip('/')
        self.model = str(g('model').value)
        self.timeout = float(g('timeout_s').value)
        self.max_steps = int(g('max_steps').value)
        # Ollama parses a STRING keep_alive as a Go duration, so "-1" fails with
        # `missing unit in duration`. A NUMBER is seconds, and -1 means never
        # evict. Accept either spelling in config and send the right JSON type.
        raw_keep = str(g('keep_alive').value).strip()
        try:
            self.keep_alive = int(raw_keep)
        except ValueError:
            self.keep_alive = raw_keep      # a duration like "30m" passes through
        self.warm_on_start = bool(g('warm_on_start').value)

        self.pub = self.create_publisher(String, 'voice_cmd', 10)
        self.status_pub = self.create_publisher(String, 'llm_state', 10)
        self.create_subscription(String, 'voice_unparsed', self._on_text, 10)

        self.busy = False
        self.get_logger().info(f'llm: {self.model} at {self.host}')
        if self.warm_on_start:
            threading.Thread(target=self._warm, daemon=True).start()

    def _warm(self):
        """Load the model into VRAM before anyone needs it."""
        self._status('warming')
        t0 = time.time()
        try:
            body = json.dumps({
                'model': self.model, 'stream': False,
                'keep_alive': self.keep_alive,
                'messages': [{'role': 'user', 'content': 'ok'}],
            }).encode()
            req = urllib.request.Request(
                f'{self.host}/api/chat', data=body,
                headers={'Content-Type': 'application/json'})
            # Generous: a cold load of a 7B is tens of seconds.
            urllib.request.urlopen(req, timeout=180).read()
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn(f'warm-up failed ({e}); the slow path will '
                                   'still work, the first command will be slow')
            self._status('unreachable', detail=str(e))
            return
        self.get_logger().info(f'model warm in {time.time() - t0:.0f}s')
        self._status('ready')

    def _on_text(self, msg):
        try:
            text = (json.loads(msg.data).get('text') or '').strip()
        except Exception:                                    # noqa: BLE001
            return
        if not text:
            return
        if self.busy:
            # One at a time. Queueing utterances would let the car act on
            # something said several seconds ago, which is worse than ignoring it.
            self.get_logger().info(f'busy, dropping: {text!r}')
            return
        threading.Thread(target=self._parse, args=(text,), daemon=True).start()

    def _status(self, state, text='', detail=''):
        self.status_pub.publish(String(data=json.dumps(
            {'state': state, 'text': text, 'detail': detail,
             'stamp': time.time()})))

    def _parse(self, text):
        self.busy = True
        self._status('thinking', text)
        t0 = time.time()
        try:
            body = json.dumps({
                'model': self.model,
                'format': SCHEMA,          # constrained decoding
                'stream': False,
                'keep_alive': self.keep_alive,
                'options': {'temperature': 0},
                'messages': [
                    {'role': 'system', 'content': SYSTEM},
                    {'role': 'user', 'content': text},
                ],
            }).encode()
            req = urllib.request.Request(
                f'{self.host}/api/chat', data=body,
                headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                payload = json.loads(r.read())
            parsed = json.loads(payload['message']['content'])
        except urllib.error.URLError as e:
            self.get_logger().warn(f'llm unreachable: {e.reason}')
            self._status('unreachable', text, str(e.reason))
            self.busy = False
            return
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn(f'llm failed: {e}')
            self._status('error', text, str(e))
            self.busy = False
            return

        dt = time.time() - t0
        self.busy = False

        if not parsed.get('understood') or not parsed.get('steps'):
            self.get_logger().info(f'llm could not express: {text!r} ({dt:.1f}s)')
            self._status('not_understood', text)
            return

        steps = []
        for st in parsed['steps'][:self.max_steps]:
            step = {'action': st['action']}
            # At most one of these; seconds wins if the model sends more.
            if st.get('seconds'):
                step['seconds'] = float(st['seconds'])
            elif st.get('metres'):
                step['metres'] = float(st['metres'])
            elif st.get('degrees'):
                step['degrees'] = float(st['degrees'])
            if st.get('until_color'):
                step['until'] = {'color': st['until_color']}
            steps.append(step)

        def describe(st):
            bits = st['action']
            if st.get('seconds'):
                bits += f"({st['seconds']:g}s)"
            if st.get('metres'):
                bits += f"({st['metres']:g}m)"
            if st.get('degrees'):
                bits += f"({st['degrees']:g}deg)"
            if st.get('until'):
                bits += f"(until {st['until']['color']})"
            return bits

        # Include the arguments, not just the action names. Reporting only the
        # actions hid a real bug: the sequence looked right while durations were
        # being dropped or attached to the wrong step.
        summary = ' -> '.join(describe(s) for s in steps)
        self.get_logger().info(f'"{text}" -> {summary} ({dt:.1f}s)')
        self._status('parsed', text, summary)

        # Same topic and shape a button press uses, so nothing downstream knows
        # or cares that a model produced this.
        self.pub.publish(String(data=json.dumps(
            {'action': 'sequence', 'steps': steps, 'source': 'llm'})))


def main(args=None):
    rclpy.init(args=args)
    node = LlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
