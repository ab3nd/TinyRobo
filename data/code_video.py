#!/usr/bin/python

#Script for coding videos 
#Uses cmd, writes csv files

import cmd
import csv
import argparse

class VideoCodeCmd(cmd.Cmd):

	def __init__(self, *args, **kwargs):
		#Call the superclass init
		cmd.Cmd.__init__(self, *args, **kwargs)

		#Argument parser for drag commands
		self.drag_parser = argparse.ArgumentParser(add_help=False, prog="drag")
		self.drag_parser.add_argument('-f', '--fingers', help="Number of fingers user used for command", type=int, default=1)
		self.drag_parser.add_argument('-h', '--hands', help="Number of hands user used for command", type=int, default=1)
		self.drag_parser.add_argument('-d', '--draw', help="What user drew with drag command", nargs='*')
		self.drag_parser.add_argument('-w', '--write', help="What user wrote with drag command", nargs='*')
		#I'm using -h and not adding help
		self.drag_parser.add_argument('--help', action='help', help='show this help message and exit')
		self.drag_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.drag_parser.add_argument('-o','--objects', help="the targets of the drag option", nargs='*')
		
		#For voice commands
		self.voice_parser = argparse.ArgumentParser(prog="voice")
		self.voice_parser.add_argument('-c', '--command', help="What user said", nargs='*')
		self.voice_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)

		#For tap commands
		self.tap_parser = argparse.ArgumentParser(prog="tap")
		self.tap_parser.add_argument('-f', '--fingers', help="Number of fingers user used for command", type=int, default=1)
		self.tap_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.tap_parser.add_argument('-c', '--count', help="how many taps", type=int, default=1)
		self.drag_parser.add_argument('-o','--objects', help="the target of the tap", nargs='*', required = True)
		
		#lasso commands
		self.lasso_parser = argparse.ArgumentParser(prog="lasso")
		self.lasso_parser.add_argument('-f', '--fingers', help="Number of fingers user used for command", type=int, default=1)
		self.lasso_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.lasso_parser.add_argument('-o','--objects', help="the target of the tap", nargs='*', required = True)
		
		#pinch commands
		self.pinch_parser = argparse.ArgumentParser(prog="pinch")
		self.pinch_parser.add_argument('-f', '--fingers', help="Number of fingers user used for command", type=int, default=1)
		self.pinch_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)

		#box selection
		self.box_parser = argparse.ArgumentParser(prog="box")
		self.box_parser.add_argument('-f', '--fingers', help="Number of fingers user used for command", type=int, default=1)
		self.box_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.box_parser.add_argument('-o','--objects', help="the target of the tap", nargs='*', required = True)
		
		#ui element
		self.ui_parser = argparse.ArgumentParser(prog="ui")
		self.ui_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.memo_parser.add_argument('-d', '--description', help="description of ui element", nargs ='*')

		#other command
		self.other_parser = argparse.ArgumentParser(prog="other")
		self.other_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.other_parser.add_argument('-d', '--description', help="description of command", nargs ='*')
		self.other_parser.add_argument('-o','--objects', help="the target of the tap", nargs='*', required = True)
		
		#memo
		self.memo_parser = argparse.ArgumentParser(prog="memo")
		self.memo_parser.add_argument('-d', '--description', help="text of the memo", nargs ='*')
		

	def do_drag(self, options):
		"""A drag consists of 'drag' fingers used, draw or write, hands used, and a sequence of targets """
		try:
			args = self.drag_parser.parse_args(options.split())
			print args.fingers
			print args.hands
			print args.draw
			print args.write
			print args.objects
			print args.time

		except SystemExit:
			return

	def complete_drag(self, options):
		pass

	def do_voice(self, options):
		try:
			args = self.voice_parser.parse_args(options.split())
			print args.command
			print args.time
		except SystemExit:
			return

	def do_tap(self, options):
		try:
			args = self.tap_parser.parse_args(options.split())
			print args.fingers
			print args.time
			print args.count
		except SystemExit:
			return

	def do_lasso(self, options):
		try:
			args = self.lasso_parser.parse_args(options.split())
		except SystemExit:
			return

	def do_pinch(self, options):
		try:
			args = self.pinch_parser.parse_args(options.split())
		except SystemExit:
			return

	def do_box(self, options):
		try:
			args = self.box_parser.parse_args(options.split())
		except SystemExit:
			return

	def do_ui(self, options):
		try:
			args = self.ui_parser.parse_args(options.split())
		except SystemExit:
			return

	def do_memo(self, options):
		try:
			args = self.memo_parser.parse_args(options.split())
			print args.text
		except SystemExit:
			return

	def do_quit(self, options):
		#TODO write the file
		exit()

if __name__ == '__main__':
	VideoCodeCmd().cmdloop()