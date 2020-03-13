# -*- coding: utf-8 -*-
# vispy: gallery 2
# Copyright (c) 2015, Vispy Development Team.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.

"""
Multiple real-time digital signals with GLSL-based clipping.
"""

from vispy import gloo, app, visuals

import numpy as np
import math
from seaborn import color_palette
from pylsl import StreamInlet, resolve_byprop
from scipy.signal import lfilter, lfilter_zi, butter
import scipy
from mne.filter import create_filter
import paho.mqtt.client as mqtt
import threading
import pandas as pd
from time import time, strftime, gmtime, sleep
import os
from pynput.keyboard import Key, Controller
# from .constants import LSL_SCAN_TIMEOUT, LSL_EEG_CHUNK

LSL_SCAN_TIMEOUT = 5
LSL_EEG_CHUNK = 12


VERT_SHADER = """
#version 120
// y coordinate of the position.
attribute float a_position;
// row, col, and time index.
attribute vec3 a_index;
varying vec3 v_index;
// 2D scaling factor (zooming).
uniform vec2 u_scale;
// Size of the table.
uniform vec2 u_size;
// Number of samples per signal.
uniform float u_n;
// Color.
attribute vec3 a_color;
varying vec4 v_color;
// Varying variables used for clipping in the fragment shader.
varying vec2 v_position;
varying vec4 v_ab;
void main() {
    float n_rows = u_size.x;
    float n_cols = u_size.y;
    // Compute the x coordinate from the time index.
    float x = -1 + 2*a_index.z / (u_n-1);
    vec2 position = vec2(x - (1 - 1 / u_scale.x), a_position);
    // Find the affine transformation for the subplots.
    vec2 a = vec2(1./n_cols, 1./n_rows)*.9;
    vec2 b = vec2(-1 + 2*(a_index.x+.5) / n_cols,
                    -1 + 2*(a_index.y+.5) / n_rows);
    // Apply the static subplot transformation + scaling.
    gl_Position = vec4(a*u_scale*position+b, 0.0, 1.0);
    v_color = vec4(a_color, 1.);
    v_index = a_index;
    // For clipping test in the fragment shader.
    v_position = gl_Position.xy;
    v_ab = vec4(a, b);
}
"""

FRAG_SHADER = """
#version 120
varying vec4 v_color;
varying vec3 v_index;
varying vec2 v_position;
varying vec4 v_ab;
void main() {
    gl_FragColor = v_color;
    // Discard the fragments between the signals (emulate glMultiDrawArrays).
    if ((fract(v_index.x) > 0.) || (fract(v_index.y) > 0.))
        discard;
    // Clipping test.
    vec2 test = abs((v_position.xy-v_ab.zw)/v_ab.xy);
    if ((test.x > 1))
        discard;
}
"""

def view(name , pos):
    print("Looking for an EEG stream...")
    streams = resolve_byprop('type', 'EEG', timeout=LSL_SCAN_TIMEOUT)

    if len(streams) == 0:
        raise(RuntimeError("Can't find EEG stream."))
    print("Start acquiring data.")
    stream_index = -1
    for idx, st in enumerate(streams):
        if st.name().find(name) != -1:
            stream_index = idx
            print("Found " + name + " at index " + str(idx) + " from " + st.name())
    inlet = StreamInlet(streams[stream_index], max_chunklen=LSL_EEG_CHUNK)
    Canvas(inlet, name= name, pos = pos)
    app.run()


class Canvas(app.Canvas):
    def __init__(self, lsl_inlet, name = "Unknown", pos = "pos", scale=500, filt=True):
        app.Canvas.__init__(self, title='EEG - ' + name,
                            keys='interactive')
        #Setup threading
        self.keyboard = Controller()
        t = threading.Thread(target=self.worker)
        t.start()
        self.status = False
        self.name = name
        self.pos = pos
        self.windowData = []
        self.previousWindowData = []
        self.alphaCounter = []
        self.freq = 256
        self.isClosed = False
        self.counter = 0
        self.temp_cal_alplha = []
        self.calibrate_alpha = 0
        self.filename = os.path.join(os.getcwd(), 'recording_' + self.name + '_' + strftime("%Y-%m-%d-%H.%M.%S", gmtime()) + '.csv')
        self.inlet = lsl_inlet
        self.isAction = False
        self.time = 0
        self.isJump = False
        info = self.inlet.info()
        description = info.desc()
        y_sig = np.sin(2 * np.pi * 8 * np.arange(256) / 256)
        y_sig = y_sig + np.sin(2 * np.pi * 9 * np.arange(256) / 256)
        y_sig = y_sig + np.sin(2 * np.pi * 10 * np.arange(256) / 256)
        y_sig = y_sig + np.sin(2 * np.pi * 11 * np.arange(256) / 256)
        y_sig = y_sig + np.sin(2 * np.pi * 12 * np.arange(256) / 256)
        self.y_sig = y_sig
        window = 10
        self.sfreq = info.nominal_srate()
        n_samples = int(self.sfreq * window)
        self.n_chans = info.channel_count()

        ch = description.child('channels').first_child()
        ch_names = [ch.child_value('label')]

        for i in range(self.n_chans):
            ch = ch.next_sibling()
            ch_names.append(ch.child_value('label'))

        # Number of cols and rows in the table.
        n_rows = self.n_chans
        n_cols = 1

        # Number of signals.
        m = n_rows * n_cols

        # Number of samples per signal.
        n = n_samples

        # Various signal amplitudes.
        amplitudes = np.zeros((m, n)).astype(np.float32)
        # gamma = np.ones((m, n)).astype(np.float32)
        # Generate the signals as a (m, n) array.
        y = amplitudes

        color = color_palette("RdBu_r", n_rows)

        color = np.repeat(color, n, axis=0).astype(np.float32)
        # Signal 2D index of each vertex (row and col) and x-index (sample index
        # within each signal).
        index = np.c_[np.repeat(np.repeat(np.arange(n_cols), n_rows), n),
                      np.repeat(np.tile(np.arange(n_rows), n_cols), n),
                      np.tile(np.arange(n), m)].astype(np.float32)

        self.program = gloo.Program(VERT_SHADER, FRAG_SHADER)
        self.program['a_position'] = y.reshape(-1, 1)
        self.program['a_color'] = color
        self.program['a_index'] = index
        self.program['u_scale'] = (1., 1.)
        self.program['u_size'] = (n_rows, n_cols)
        self.program['u_n'] = n

        # text
        self.font_size = 48.
        self.names = []
        self.quality = []
        for ii in range(self.n_chans):
            text = visuals.TextVisual(ch_names[ii], bold=True, color='white')
            self.names.append(text)
            text = visuals.TextVisual('', bold=True, color='white')
            self.quality.append(text)

        self.quality_colors = color_palette("RdYlGn", 11)[::-1]

        self.scale = scale
        self.n_samples = n_samples
        self.filt = filt
        self.af = [1.0]

        self.data_f = np.zeros((n_samples, self.n_chans))
        self.data = np.zeros((n_samples, self.n_chans))

        self.bf = create_filter(self.data_f.T, self.sfreq, 8, 15.,
                                method='fir')

        zi = lfilter_zi(self.bf, self.af)
        self.filt_state = np.tile(zi, (self.n_chans, 1)).transpose()

        self._timer = app.Timer('auto', connect=self.on_timer, start=True)
        gloo.set_viewport(0, 0, *self.physical_size)
        gloo.set_state(clear_color='black', blend=True,
                       blend_func=('src_alpha', 'one_minus_src_alpha'))

        self.show()

    def on_key_press(self, event):

        # toggle filtering
        if event.key.name == 'D':
            self.filt = not self.filt

        # increase time scale
        if event.key.name in ['+', '-']:
            if event.key.name == '+':
                dx = -0.05
            else:
                dx = 0.05
            scale_x, scale_y = self.program['u_scale']
            scale_x_new, scale_y_new = (scale_x * math.exp(1.0 * dx),
                                        scale_y * math.exp(0.0 * dx))
            self.program['u_scale'] = (
                max(1, scale_x_new), max(1, scale_y_new))
            self.update()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe("topic/parameters")
        #self.client.subscribe("topic/test")
        # The callback for when a PUBLISH message is received from the server.

    def worker(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect("localhost", 1883, 60)
        self.client.on_message = self.on_message
        # Blocking call that processes network traffic, dispatches callbacks and
        # handles reconnecting.
        # Other loop*() functions are available that give a threaded interface and a
        # manual interface.
        self.client.loop_forever()
        return

    def on_mouse_wheel(self, event):
        dx = np.sign(event.delta[1]) * .05
        scale_x, scale_y = self.program['u_scale']
        scale_x_new, scale_y_new = (scale_x * math.exp(0.0 * dx),
                                    scale_y * math.exp(2.0 * dx))
        self.program['u_scale'] = (max(1, scale_x_new), max(0.01, scale_y_new))
        self.update()

    def on_timer(self, event):
        """Add some data at the end of each signal (real-time signals)."""

        samples, timestamps = self.inlet.pull_chunk(timeout=0.0,
                                                    max_samples= 80 )
        if timestamps:
            # for sp in samples:
                # print(sp)

            samples = np.array(samples)[:, ::-1]

            self.data = np.vstack([self.data, samples])
            self.data = self.data[-self.n_samples:]
            filt_samples, self.filt_state = lfilter(self.bf, self.af, samples,
                                                    axis=0, zi=self.filt_state)
            self.data_f = np.vstack([self.data_f, filt_samples])
            self.data_f = self.data_f[-self.n_samples:]      

            if self.filt:
                plot_data = self.data_f / self.scale
            elif not self.filt:
                plot_data = (self.data - self.data.mean(axis=0)) / self.scale

            sd = np.std(plot_data[-int(self.sfreq):],
                        axis=0)[::-1] * self.scale
            co = np.int32(np.tanh((sd - 30) / 15) * 5 + 5)
            for ii in range(self.n_chans):
                self.quality[ii].text = '%.2f' % (sd[ii])
                self.quality[ii].color = self.quality_colors[co[ii]]
                self.quality[ii].font_size = 12 + co[ii]

                self.names[ii].font_size = 12 + co[ii]
                self.names[ii].color = self.quality_colors[co[ii]]

            self.program['a_position'].set_data(
                plot_data.T.ravel().astype(np.float32))
            self.update()  
             
            t = time()
            if(t - self.time > 2.0 and not self.isJump):
                if(float(self.quality[1].text) >= 30):
                    print('jump! ', t)
                    self.time = time()
                    if(self.pos == "left"):
                        self.keyboard.press('q')
                        self.keyboard.release('q')
                        self.keyboard.press('w')
                        self.keyboard.release('w')
                        self.keyboard.press('e')
                        self.keyboard.release('e')
                        self.keyboard.press('r')
                        self.keyboard.release('r')
                        self.keyboard.press('a')
                        self.keyboard.release('a')
                        self.keyboard.press('s')
                        self.keyboard.release('s')
                        self.keyboard.press('d')
                        self.keyboard.release('d')
                        self.keyboard.press('f')
                        self.keyboard.release('f')
                        self.keyboard.press('z')
                        self.keyboard.release('z')
                        self.keyboard.press('x')
                        self.keyboard.release('x')
                        self.keyboard.press('c')
                        self.keyboard.release('c')
                        self.keyboard.press('v')
                        self.keyboard.release('v')
                    elif(self.pos == "right"):
                        self.keyboard.press('t')
                        self.keyboard.release('t')
                        self.keyboard.press('g')
                        self.keyboard.release('g')
                        self.keyboard.press('y')
                        self.keyboard.release('y')
                        self.keyboard.press('u')
                        self.keyboard.release('u')
                        self.keyboard.press('i')
                        self.keyboard.release('i')
                        self.keyboard.press('o')
                        self.keyboard.release('o')
                        self.keyboard.press('p')
                        self.keyboard.release('p')
                        self.keyboard.press('h')
                        self.keyboard.release('h')
                        self.keyboard.press('j')
                        self.keyboard.release('j')
                        self.keyboard.press('k')
                        self.keyboard.release('k')
                        self.keyboard.press('l')
                        self.keyboard.release('l')
                        self.keyboard.press('b')
                        self.keyboard.release('b')
                        self.keyboard.press('n')
                        self.keyboard.release('n')
                        self.keyboard.press('m')
                        self.keyboard.release('m')
                    self.isJump = True
            elif(float(self.quality[1].text) < 30):
                self.isJump = False      

    def on_resize(self, event):
        # Set canvas viewport and reconfigure visual transforms to match.
        vp = (0, 0, self.physical_size[0], self.physical_size[1])
        self.context.set_viewport(*vp)

        for ii, t in enumerate(self.names):
            t.transforms.configure(canvas=self, viewport=vp)
            t.pos = (self.size[0] * 0.025,
                     ((ii + 0.5) / self.n_chans) * self.size[1])

        for ii, t in enumerate(self.quality):
            t.transforms.configure(canvas=self, viewport=vp)
            t.pos = (self.size[0] * 0.975,
                     ((ii + 0.5) / self.n_chans) * self.size[1])

    def on_draw(self, event):
        gloo.clear()
        gloo.set_viewport(0, 0, *self.physical_size)
        self.program.draw('line_strip')
        [t.draw() for t in self.names + self.quality]

    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return b, a


    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def on_message(self, client, userdata, message):
        print("message received " ,str(message.payload.decode("utf-8")))
        print("message topic=",message.topic)
        if message.topic == "topic/parameters":
            if str(message.payload.decode("utf-8")) == 'action':
                print('found action command')
                self.isAction = True
            else:
                self.isAction = False
