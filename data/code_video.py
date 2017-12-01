#!/usr/bin/python

"""
Script for coding videos 

Coding files are stored as JSON data. 

{
	participant=<number>,
	tasks={
		1=[event, event, event, ...]
		2=[event, event, event,...]
		...
	}
}
"""

import cmd
import csv
import argparse
import json



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
		#I'm using -h and not adding help, so add it explicitly
		self.drag_parser.add_argument('--help', action='help', help='show this help message')
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
		self.tap_parser.add_argument('-o','--objects', help="the target of the tap", nargs='*', required = True)
		
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
		self.box_parser.add_argument('-s', '--start', help="start point of selection", choices=["tl", "tr", "bl", "br"])
		
		#ui element
		self.ui_parser = argparse.ArgumentParser(prog="ui")
		self.ui_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.ui_parser.add_argument('-d', '--description', help="description of ui element", nargs ='*')

		#other command
		self.other_parser = argparse.ArgumentParser(prog="other")
		self.other_parser.add_argument('-t', '--time', help="timestamp of action",type=float, required=True)
		self.other_parser.add_argument('-d', '--description', help="description of command", nargs ='*', required = True)
		self.other_parser.add_argument('-o','--objects', help="the target of the command", nargs='*', required = True)
		
		#memo
		self.memo_parser = argparse.ArgumentParser(prog="memo")
		self.memo_parser.add_argument('-d', '--description', help="text of the memo", nargs ='*')
		
		#participant
		self.participant_number = None
		self.out_file = None
		self.part_parser = argparse.ArgumentParser(prog="participant")
		self.part_parser.add_argument('number', help="participant number", type=int)

		#task
		self.task_number = None
		self.task_parser = argparse.ArgumentParser(prog="participant")
		self.task_parser.add_argument('number', help="participant number", type=int)

		self.state={}

	def do_drag(self, options):
		"""A drag consists of 'drag' fingers used, draw or write, hands used, and a sequence of targets """
		try:
			args = self.drag_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_drag(self):
		self.drag_parser.print_help()

	def do_voice(self, options):
		try:
			args = self.voice_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_voice(self):
		self.voice_parser.print_help()

	def do_tap(self, options):
		try:
			args = self.tap_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_tap(self):
		self.tap_parser.print_help()

	def do_lasso(self, options):
		try:
			args = self.lasso_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_lasso(self):
		self.lasso_parser.print_help()

	def do_pinch(self, options):
		try:
			args = self.pinch_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_pinch(self):
		self.pinch_parser.print_help()

	def do_box(self, options):
		try:
			args = self.box_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_box(self):
		self.box_parser.print_help()

	def do_ui(self, options):
		try:
			args = self.ui_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_ui(self):
		self.ui_parser.print_help()

	def do_memo(self, options):
		try:
			args = self.memo_parser.parse_args(options.split())
			self.state["tasks"][self.task_number].append(args.__dict__)
		except SystemExit:
			return

	def help_memo(self):
		self.memo_parser.print_help()

	def do_quit(self, options):
		self.write_state()
		exit()

	def do_participant(self, options):
		"""Start a new participant, which will create a new log file"""
		try:
			args = self.part_parser.parse_args(options.split())

			#We have a previous participant. 
			if self.participant_number is not None:
				if self.out_file is not None:
					self.write_state()
					self.out_file.close()

			self.participant_number = args.number
			print "Starting new file for participant {0}".format(self.participant_number)
			#TODO this will clobber the file
			self.out_file = open("p_{0}.json".format(self.participant_number), 'w')

			#Set up the new state
			self.state = {}
			self.state["participant"] = self.participant_number
			self.state["tasks"] = {}
			
		except SystemExit:
			return

	def do_task(self, options):
		"""Set the task to which events will be appended"""
		try:
			args = self.task_parser.parse_args(options.split())
			self.task_number = args.number
			if self.task_number in self.state["tasks"].keys():
				#We already have that task, so no need to clear it
				print "Switched to task {0}".format(self.task_number)
			else:
				print "Added task {0}".format(self.task_number)
				self.state["tasks"][self.task_number] = []
		except SystemExit:
			return

	def write_state(self):
		if self.out_file is not None:
			#Clear out the old stuff first
			self.out_file.truncate(0)
			self.out_file.write(json.dumps(self.state))

	def precmd(self, line):
		#So quit still works even if things are not set
		if line.startswith("quit"):
			return cmd.Cmd.precmd(self, line)

		#Check if participant number and task number are set, and demand that they be set
		#if they are not. 
		if self.participant_number is None:
			if line.startswith("participant"):
				return line
			else:
				print "Set participant number with 'participant <number>'"
				return ""

		if self.task_number is None:
			if line.startswith("task"):
				return line
			else:
				print "Set task number with 'task <number>'"
				return ""

		else:
			#We're good, call the superclass version
			return cmd.Cmd.precmd(self, line)

	#Actually want to do nothing, not execute the last command
	def emptyline(self):
		pass

	def postcmd(self, stop, line):
		self.write_state()
		return cmd.Cmd.postcmd(self, stop, line)

if __name__ == '__main__':
	VideoCodeCmd().cmdloop()