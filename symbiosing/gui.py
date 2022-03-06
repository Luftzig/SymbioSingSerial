from collections import namedtuple
from logging import debug
from typing import Literal, Union

import asyncio
import wx
import os
from wxasync import WxAsyncApp, AsyncBind, StartCoroutine

from symbiosing.orchestrator import Orchestrator

HasFile = namedtuple('HasFile', ['file'])
Connected = namedtuple('Connected', [])
Ready = namedtuple('Ready', ['file'])


class MainFrame(wx.Frame):
    status: Union[Literal['loaded'], HasFile, Connected, Ready] = 'loaded'

    def __init__(self, parent=None):
        super(MainFrame, self).__init__(parent)
        self.orchestrator = Orchestrator()
        self.current_dir = os.getcwd()
        panel = wx.Panel(self)
        text = wx.StaticText(panel, label="SymbioSing Serial")
        vbox = wx.BoxSizer(wx.VERTICAL)
        self.status_text = wx.StaticText(panel)
        self.load_button = wx.Button(panel, label="Load Schedule")
        self.connect_button = wx.Button(panel, label="Connect")
        self.start_button = wx.Button(panel, label="Start")
        vbox.Add(text, 1)
        vbox.Add(self.status_text, 1)
        vbox.AddStretchSpacer(1)
        vbox.Add(self.load_button, 1)
        vbox.Add(self.connect_button, 1)
        vbox.Add(self.start_button, 1)
        self.start_button.Hide()
        panel.SetSizer(vbox)
        self.Layout()
        self.Bind(wx.EVT_BUTTON, self.on_load, self.load_button)
        self.Bind(wx.EVT_BUTTON, self.on_connect, self.connect_button)
        AsyncBind(wx.EVT_BUTTON, self.start, self.start_button)

    def on_load(self, event):
        dlg = wx.FileDialog(self,
                            message="Choose file",
                            defaultDir=self.current_dir,
                            wildcard="*.json",
                            style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST | wx.FD_CHANGE_DIR)
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self._file_loaded(path)
            self.current_dir = path
        dlg.Destroy()

    def on_connect(self, event):
        self.orchestrator.connect_devices()
        if isinstance(self.status, Connected) or self.status == 'loaded':
            self.status = Connected()
        elif isinstance(self.status, HasFile):
            self.status = Ready(file=self.status.file)
        elif isinstance(self.status, Ready):
            pass
        self._update_status()

    def _file_loaded(self, file):
        self.orchestrator.load_file(file)
        if isinstance(self.status, HasFile) or self.status == 'loaded':
            self.status = HasFile(file=file)
        elif isinstance(self.status, Connected):
            self.status = Ready(file=file)
        elif isinstance(self.status, Ready):
            self.status.file = file
        self._update_status()

    def _update_status(self):
        def connected_text():
            return ('Connected to {devices} devices:'.format(devices=len(self.orchestrator.devices))
                    + '\n'
                    + ',\t'.join(d.name for d in self.orchestrator.devices)
                    )

        def schedule_text():
            return 'Schedule Loaded: ' + self.status.file

        self.status_text.SetLabel(
            '' if self.status == 'loaded'
            else schedule_text() if isinstance(self.status, HasFile)
            else connected_text() if isinstance(self.status, Connected)
            else '\n'.join([schedule_text(), connected_text()])
        )
        if isinstance(self.status, Ready):
            self.start_button.Show()
            self.Layout()
        else:
            self.start_button.Hide()

    async def start(self, event):
        print('starting playback')
        await self.orchestrator.start()


def show():
    app = WxAsyncApp()
    frame = MainFrame()
    frame.Show()
    app.SetTopWindow(frame)
    return app.MainLoop()
