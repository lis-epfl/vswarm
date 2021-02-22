#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import psutil


class Processes:

    def __init__(self, process_name):
        self.processes = []
        self.process_name = process_name
        self._suspended = False

    def suspend(self):
        if not self._suspended:
            for proc in psutil.process_iter():
                if self.process_name in proc.name():
                    self.processes.append(proc)
                    proc.suspend()
            self._suspended = True

    def resume(self):
        if self._suspended:
            for proc in self.processes:
                proc.resume()
            self._suspended = False
