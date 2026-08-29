#!/usr/bin/env python3
"""On-robot speech: USB microphone -> /voice_cmd, and -> /voice_unparsed.

The close-range half of the voice design. Walk up to the car and talk to it, no
phone required. The phone covers distance; this covers standing next to it.
Both publish the same message to the same topic, so nothing downstream knows or
cares which one spoke.

Recognition is Vosk, running entirely offline on the Pi, for three reasons:

* **Latency.** faster-whisper is more accurate on free speech but takes one to
  two seconds per utterance on a Pi 5 CPU. For "stop" that is a safety problem,
  not an inconvenience.
* **No internet.** The whole point of the on-robot path is that it works without
  a phone or a network. Cloud recognition would reintroduce the dependency it
  exists to avoid.
* **Constrained grammar.** Vosk accepts an explicit list of the only phrases that
  exist, and then cannot return anything else. In a lab full of motors and other
  people's conversations that matters more than model quality: a fragment of
  someone else's sentence cannot become a command, because the words are not in
  the vocabulary.

The grammar is generated from behavior_node's own keyword table, so the two
cannot drift apart. Set `use_grammar: false` for free-form recognition, which is
what the LLM tier will eventually want.
"""
import json
import os
import queue
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Mirrors behavior_node.KEYWORDS. Vosk's grammar is a WORD list, so it can and
# will return any combination of these words, including single words drawn out of
# multi-word phrases ("back" from "back up"). behavior_node's matcher therefore
# accepts the single-word forms too.
PHRASES = [
    # Wake word. The mic listens continuously and the rest of this vocabulary is
    # ordinary English, so behavior_node requires this prefix on mic commands.
    # Two syllables on purpose: a longer word gives the recogniser more acoustic
    # content than a one-syllable name like "rex", which also rhymes with several
    # common words. Note "atlas" is a Boston Dynamics robot, so it does come up in
    # robotics conversation; if phantom commands ever appear, suspect that first.
    'atlas',
    'stop', 'halt',
    'wait', 'hold', 'stay', 'freeze',
    'forward', 'go forward', 'go straight', 'ahead',
    'reverse', 'back up', 'go back', 'backward', 'backwards',
    'circle right', 'circle to the right',
    'circle left', 'circle to the left',
    'figure', 'eight', 'figure eight',
    'follow', 'follow me', 'come here',
    # Chain separators, so a spoken sequence survives the constrained grammar:
    # "atlas forward then circle left then stop".
    'then', 'and',
    # Conditional steps: "atlas forward until green then stop".
    'until', 'green', 'red', 'you', 'see', 'the',
]


class SttNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        self.declare_parameter('model_path', '/home/pi/models/vosk-small-en')
        self.declare_parameter('device', '')        # '' = ALSA default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('use_grammar', True)
        self.declare_parameter('min_words', 1)
        # A SECOND, unconstrained recogniser on the same audio, used only when
        # the grammar produced nothing. The grammar cannot emit words outside the
        # command vocabulary, which is exactly what makes it reliable and also
        # what makes it useless for the LLM tier: it can never produce "for five
        # seconds". Free-form fills that gap without weakening the fast path.
        #
        # Deliberately NOT merged into one transcript. The keyword matcher is
        # substring-based, so a free-form "drive forward for five seconds then
        # spin left twice" contains "forward" and would match a plain forward
        # primitive, silently running a fragment of what was said.
        self.declare_parameter('freeform', True)
        # The free-form pass must clear the same gate the grammar pass does. An
        # unconstrained recogniser on an always-on microphone invents words from
        # room noise ("surrender willing case"), and every one of those would be
        # a needless LLM call. The wake word is what separates speech aimed at
        # the car from speech that merely happened near it.
        self.declare_parameter('freeform_wake', 'atlas')
        self.declare_parameter('freeform_min_words', 3)

        g = self.get_parameter
        self.model_path = str(g('model_path').value)
        self.device = str(g('device').value) or None
        self.rate = int(g('sample_rate').value)
        self.use_grammar = bool(g('use_grammar').value)
        self.min_words = int(g('min_words').value)
        self.freeform = bool(g('freeform').value)
        self.freeform_wake = str(g('freeform_wake').value).lower().strip()
        self.freeform_min_words = int(g('freeform_min_words').value)

        self.pub = self.create_publisher(String, 'voice_cmd', 10)
        self.heard_pub = self.create_publisher(String, 'stt_heard', 10)
        # Straight to the slow path, bypassing the keyword matcher entirely.
        self.unparsed_pub = self.create_publisher(String, 'voice_unparsed', 10)

        self.audio = queue.Queue()
        self.running = True
        # Set when the device refuses the requested format and we fall back.
        self.in_rate = self.rate
        self.in_channels = 1
        self._resample_state = None

    # -- audio ----------------------------------------------------------------

    def _cb(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'audio: {status}')
        self.audio.put(bytes(indata))

    def _to_vosk(self, data):
        """Convert whatever the device gave us into mono 16 kHz PCM.

        USB mics frequently refuse 16 kHz mono and only offer their native rate
        and channel count. Rather than fail, capture what the device wants and
        convert here: downmix to mono, then resample. `audioop.ratecv` carries
        state between chunks, which matters or every block boundary clicks.
        """
        import audioop
        if self.in_channels == 2:
            data = audioop.tomono(data, 2, 0.5, 0.5)
        if self.in_rate != self.rate:
            data, self._resample_state = audioop.ratecv(
                data, 2, 1, self.in_rate, self.rate, self._resample_state)
        return data

    def _open_stream(self, sd, dev):
        """Open at the rate Vosk wants, or fall back to the device's own."""
        try:
            stream = sd.RawInputStream(
                samplerate=self.rate, blocksize=8000, device=dev,
                dtype='int16', channels=1, callback=self._cb)
            self.in_rate, self.in_channels = self.rate, 1
            self.get_logger().info(f'audio: {self.rate} Hz mono, native')
            return stream
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn(f'{self.rate} Hz mono refused ({e}); '
                                   'falling back to device native')

        info = sd.query_devices(dev, 'input')
        self.in_rate = int(info['default_samplerate'])
        self.in_channels = min(2, int(info['max_input_channels']))
        stream = sd.RawInputStream(
            samplerate=self.in_rate, blocksize=int(self.in_rate / 2),
            device=dev, dtype='int16', channels=self.in_channels,
            callback=self._cb)
        self.get_logger().info(
            f'audio: {self.in_rate} Hz x{self.in_channels}, converting to '
            f'{self.rate} Hz mono in software')
        return stream

    def _escalate_freeform(self, free_text):
        """Send a free-form transcript to the LLM, if it was aimed at the car."""
        low = free_text.lower()
        if self.freeform_wake and self.freeform_wake not in low:
            self.get_logger().debug(f'free-form ignored (no wake): {free_text!r}')
            return
        # Strip the name, then require enough left over to be a real request.
        body = low.replace(self.freeform_wake, ' ', 1).strip()
        if len(body.split()) < self.freeform_min_words:
            self.get_logger().debug(f'free-form too short: {free_text!r}')
            return
        self.get_logger().info(f'free-form: {body!r} -> llm')
        self.heard_pub.publish(String(data=f'(freeform) {body}'))
        self.unparsed_pub.publish(String(data=json.dumps(
            {'text': body, 'source': 'onboard-mic-freeform'})))

    def listen(self):
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model, SetLogLevel

        SetLogLevel(-1)                       # Vosk is extremely chatty otherwise

        if not os.path.isdir(self.model_path):
            self.get_logger().error(
                f'no Vosk model at {self.model_path}. Download a small model '
                'and unpack it there.')
            return

        model = Model(self.model_path)
        if self.use_grammar:
            grammar = json.dumps(PHRASES + ['[unk]'])
            rec = KaldiRecognizer(model, self.rate, grammar)
            self.get_logger().info(f'grammar-constrained, {len(PHRASES)} phrases')
        else:
            rec = KaldiRecognizer(model, self.rate)
            self.get_logger().info('free-form recognition')
        rec.SetWords(False)

        # Second pass, sharing the model (only the recogniser state is per-pass,
        # so this costs decoding time rather than another 68 MB).
        free = None
        if self.freeform and self.use_grammar:
            free = KaldiRecognizer(model, self.rate)
            free.SetWords(False)
            self.get_logger().info('free-form second pass enabled (for the LLM)')

        dev = None
        if self.device:
            dev = int(self.device) if self.device.isdigit() else self.device

        with self._open_stream(sd, dev):
            self.get_logger().info('listening')
            while self.running and rclpy.ok():
                try:
                    data = self.audio.get(timeout=0.5)
                except queue.Empty:
                    continue
                data = self._to_vosk(data)
                final = rec.AcceptWaveform(data)
                free_final = free.AcceptWaveform(data) if free else False
                if not final:
                    continue
                text = json.loads(rec.Result()).get('text', '').strip()
                free_text = ''
                if free:
                    # Flush whether or not the free pass called it final, so both
                    # decoders report on the same utterance boundary.
                    free_text = json.loads(
                        free.Result() if free_final else free.FinalResult()
                    ).get('text', '').strip()
                if not text or text == '[unk]':
                    # Grammar heard nothing it recognises. If the free pass heard
                    # words, hand them to the LLM rather than discarding them.
                    if free_text:
                        self._escalate_freeform(free_text)
                    continue
                # Strip the unknown-token filler the grammar emits for anything
                # outside the vocabulary, so "uhh stop" still reads as "stop".
                text = ' '.join(w for w in text.split() if w != '[unk]')
                if len(text.split()) < self.min_words:
                    continue

                self.heard_pub.publish(String(data=text))
                self.get_logger().info(f'heard: {text!r}')
                # Publish the TEXT, not an intent. behavior_node owns the
                # vocabulary, so the microphone and the phone stay interchangeable.
                self.pub.publish(String(data=json.dumps(
                    {'text': text, 'source': 'onboard-mic'})))


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.listen()
    except KeyboardInterrupt:
        pass
    except Exception as e:                                   # noqa: BLE001
        node.get_logger().error(f'stt failed: {e}')
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
