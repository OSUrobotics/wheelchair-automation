#!/usr/bin/env python

class ComputerSim(object):

	def __init__(self):
		self.latency = 1
		self.cpu = 1

	def set_latency(self, val):
		self.latency = val

	def set_cpu(self, val):
		self.cpu = val