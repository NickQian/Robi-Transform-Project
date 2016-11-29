#!/usr/bin/env python

"""
    pi_trees_lib.py - Version 0.1 2013-08-28
    
    Core classes for implementing Behavior Trees in Python
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import string
import random

import os

import pygraphviz as pgv

class TaskStatus(object):
    """ A class for enumerating task statuses """
    FAILURE = 0
    SUCCESS = 1
    RUNNING = 2

# A global value to track when the tree's status has changed.  Used in the print_dot_tree() function.
last_dot_tree = ''
    
class Task(object):
    """ "The base Task class """
    def __init__(self, name, children=None, reset_after=False, announce=False, *args, **kwargs):
        self.name = name
        self.status = None
        self.reset_after = reset_after
        self._announce = announce
                
        if children is None:
            children = []
            
        self.children = children
                         
    def run(self):
        pass

    def reset(self):
        for c in self.children:
            c.reset()
            
        self.status = None

    def add_child(self, c):
        self.children.append(c)

    def remove_child(self, c):
        self.children.remove(c)
        
    def prepend_child(self, c):
        self.children.insert(0, c)
        
    def insert_child(self, c, i):
        self.children.insert(i, c)
        
    def get_status(self):
        return self.status
    
    def set_status(self, s):
        self.status = s
    
    def announce(self):
        print("Executing task " + str(self.name))
        
    def get_type(self):
        return type(self)
    
    # These next two functions allow us to use the 'with' syntax
    def __enter__(self):
        return self.name
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if  exc_type is not None:
            return False
        return True
    
class Selector(Task):
    """ A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(Selector, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            print ("DBG:c in (Selector) is:", c)    #NickQian
            c.status = c.run()
            
            if c.status != TaskStatus.FAILURE:
                if c.status == TaskStatus.SUCCESS:
                    if self.reset_after:
                        self.reset()
                        return self.status
                    else:
                        return c.status
                return c.status
            
        return TaskStatus.FAILURE
 
class Sequence(Task):
    """
        A sequence runs each task in order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(Sequence, self).__init__(name, *args, **kwargs)
 
    def run(self):
        if self._announce:
            self.announce()
            
        for c in self.children:
            print ("DBG: c in (Sequence) is:", c)    #NickQian
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS:
                if c.status == TaskStatus.FAILURE:
                    if self.reset_after:
                        self.reset()
                return c.status   
        
        if self.reset_after:
            self.reset()
        
        return TaskStatus.SUCCESS
    
class RandomSelector(Task):
    """ A selector runs each task in random order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomSelector, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True
                    
        for c in self.children:
            
            c.status = c.run()
            
            if c.status != TaskStatus.FAILURE:
                if c.status == TaskStatus.SUCCESS:
                    self.shuffled = False

                return c.status

        self.shuffled = False
        
        if self.reset_after:
            self.reset()

        return TaskStatus.FAILURE
    
class RandomSequence(Task):
    """
        A sequence runs each task in random order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomSequence, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True
        
        for c in self.children:
            
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS:
                if c.status == TaskStatus.FAILURE:
                    self.shuffled = False
                    
                return c.status   

        self.shuffled = False
        
        if self.reset_after:
            self.reset()

        return TaskStatus.SUCCESS
    
class Iterator(Task):
    """
        Iterate through all child tasks ignoring failure.
    """
    def __init__(self, name, *args, **kwargs):
        super(Iterator, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
                        
            c.status = c.run()
                         
            if c.status == TaskStatus.RUNNING:
                return c.status
            
        if self.reset_after:
            self.reset()
            
        return TaskStatus.SUCCESS
    
class RandomIterator(Task):
    """
        Iterate through all child tasks randomly (without replacement) ignoring failure.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomIterator, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True

        for c in self.children:
            
            c.status = c.run()
                         
            if c.status == TaskStatus.RUNNING:
                return c.status
            
        self.shuffled = False
        
        if self.reset_after:
            self.reset()

        return TaskStatus.SUCCESS
    
class ParallelOne(Task):
    """
        A parallel task runs each child task at (roughly) the same time.
        The ParallelOne task returns success as soon as any child succeeds.
    """
    def __init__(self, name, *args, **kwargs):
        super(ParallelOne, self).__init__(name, *args, **kwargs)
        
        self.failure = dict()
        self.index = 0
                 
    def run(self):
        n_children = len(self.children)

        if self.index < n_children:
            child = self.children[self.index]
            print ("DBG: child in (ParallelOne) is:", child)    #NickQian
            child.status = child.run()
            
            if child.status != TaskStatus.SUCCESS:
                self.index += 1
                
                if child.status == TaskStatus.FAILURE:
                    self.failure[child.name] = TaskStatus.FAILURE
                    
                return TaskStatus.RUNNING
            else:
                if self.reset_after:
                    self.reset()
                    self.index = 0
                return TaskStatus.SUCCESS

        elif len(self.failure) == n_children:
            if self.reset_after:
                self.reset()
            return TaskStatus.FAILURE
        else:
            self.index = 0
            return TaskStatus.RUNNING
        
    def reset(self):
        super(ParallelOne, self).reset()
        self.failure = dict()
        self.index = 0
        
class ParallelAll(Task):
    """
        A parallel task runs each child task at (roughly) the same time.
        The ParallelAll task requires all subtasks to succeed for it to succeed.
    """
    def __init__(self, name, *args, **kwargs):
        super(ParallelAll, self).__init__(name, *args, **kwargs)
        
        self.success = dict()
        self.index = 0
                 
    def run(self):
        n_children = len(self.children)
        
        if self.index < n_children:
            child = self.children[self.index]
            child.status = child.run()
        
            if child.status != TaskStatus.FAILURE:
                self.index += 1
                
                if child.status == TaskStatus.SUCCESS:
                    self.success[child.name] = TaskStatus.SUCCESS
                    
                return TaskStatus.RUNNING
            else:
                if self.reset_after:
                    self.reset()
                return TaskStatus.FAILURE

        elif len(self.success) == n_children:
            if self.reset_after:
                self.reset()
            return TaskStatus.SUCCESS
        else:
            self.index = 0
            return TaskStatus.RUNNING
        
    def reset(self):
        super(ParallelAll, self).reset()
        self.success = dict()
        self.index = 0
        
class Loop(Task):
    """
        Loop over one or more subtasks for the given number of iterations
        Use the value -1 to indicate a continual loop.
    """
    def __init__(self, name, announce=True, *args, **kwargs):
        super(Loop, self).__init__(name, *args, **kwargs)
        
        self.iterations = kwargs['iterations']
        self._announce = announce
        self.loop_count = 0
        self.name = name
        print("Loop iterations: " + str(self.iterations))
        
    def run(self):
        
        while True:
            if self.iterations != -1 and self.loop_count >= self.iterations:
                return TaskStatus.SUCCESS
                        
            for c in self.children:
                while True:
                    c.status = c.run()
                    
                    if c.status == TaskStatus.SUCCESS:
                        break

                    return c.status
                
                c.reset()
                
            self.loop_count += 1
            
            if self._announce:
                print(self.name + " COMPLETED " + str(self.loop_count) + " LOOP(S)")
                
class Limit(Task):
    """
        Limit the number of times a task can execute
    """
    def __init__(self, name, announce=True, *args, **kwargs):
        super(Limit, self).__init__(name, *args, **kwargs)
        
        self.max_executions = kwargs['max_executions']
        self._announce = announce
        self.execution_count = 0
        self.name = name
        print("Limit number of executions to: " + str(self.max_executions))
        
    def run(self):
        if self.execution_count >= self.max_executions:
            self.execution_count = 0
            
            if self._announce:
                print(self.name + " reached maximum number (" + str(self.max_executions) + ") of executions.")
                
            return TaskStatus.FAILURE
                    
        for c in self.children:
            c.status = c.run()
            self.execution_count += 1
            return c.status


class IgnoreFailure(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, name, *args, **kwargs):
        super(IgnoreFailure, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            else:
                return c.status

        return TaskStatus.SUCCESS
    
    
class AlwaysFail(Task):
    """
        Always return FAILURE
    """
    def __init__(self, name, *args, **kwargs):
        super(AlwaysFail, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            return c.status

        return TaskStatus.FAILURE
    
class AlwaysSucceed(Task):
    """
        Always return SUCCESS
    """
    def __init__(self, name, *args, **kwargs):
        super(AlwaysSucceed, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            return c.status

        return TaskStatus.SUCCESS
    
    
class Invert(Task):
    """
        Turn SUCCESS into FAILURE and vice-versa
    """
    def __init__(self, name, *args, **kwargs):
        super(Invert, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            elif c.status == TaskStatus.SUCCESS:
                return TaskStatus.FAILURE
            
            else:
                return c.status

# Alias TaskNot to Invert for backwards compatibility
TaskNot = Invert

class UntilFail(Task):
    """
        Continue executing a task until it fails
    """
    def __init__(self, name, *args, **kwargs):
        super(UntilFail, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                break
            
            else:
                return c.status
        
        return TaskStatus.SUCCESS
    
class AutoRemoveSequence(Task):
    """ 
        Remove each successful subtask from a sequence 
    """
    def __init__(self, name, *args, **kwargs):
        super(AutoRemoveSequence, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.FAILURE
            
            if c.statuss == TaskStatus.RUNNING:
                return TaskStatus.RUNNING
        
            try:
                self.children.remove(self.children[0])
            except:
                return TaskStatus.FAILURE
                
        return TaskStatus.SUCCESS
    
class CallbackTask(Task):
    """ 
        Turn any callback function (cb) into a task
    """
    def __init__(self, name, cb=None, cb_args=[], cb_kwargs={}, **kwargs):
        super(CallbackTask, self).__init__(name, cb=None, cb_args=[], cb_kwargs={}, **kwargs)
        
        self.name = name
        self.cb = cb
        self.cb_args = cb_args
        self.cb_kwargs = cb_kwargs
  
    def run(self):
        status = self.cb(*self.cb_args, **self.cb_kwargs)
                
        if status is None:
            self.status = TaskStatus.RUNNING
         
        elif status:
            self.status = TaskStatus.SUCCESS
 
        else:
            self.status = TaskStatus.FAILURE
            
        return self.status
    
    def reset(self):
        self.status = None
        
def WaitTask(Task):
    """
        This is a *blocking* wait task.  The interval argument is in seconds.
    """
    def __init__(self, name, interval, *args, **kwargs):
        super(WaitTask, self).__init__(name, interval, *args, **kwargs)
 
    def run(self):
        sleep(interval)

        return TaskStatus.SUCCESS

class loop(Task):
    """
        Loop over one or more subtasks a given number of iterations
    """
    def __init__(self, task, iterations=-1):
        new_name = task.name + "_loop_" + str(iterations)
        super(loop, self).__init__(new_name)

        self.iterations = iterations
        self.old_run = task.run
        self.old_reset = task.reset
        self.old_children = task.children
        self.loop_count = 0
        
        print("Loop iterations: " + str(self.iterations))
        
    def run(self):
        if self.iterations != -1 and self.loop_count >= self.iterations:
            return TaskStatus.SUCCESS

        print("Loop " + str(self.loop_count + 1))
            
        while True:
            self.status = self.old_run()
            
            if self.status == TaskStatus.SUCCESS:
                break
            else:
                return self.status
                
        self.old_reset()
        self.loop_count += 1
    
class limit(Task):
    """
        Limit a task to the given number of executions
    """
    def __init__(self, task, max_executions=-1):
        new_name = task.name + "_limit_" + str(max_executions)
        super(limit, self).__init__(new_name)

        self.max_executions = max_executions
        self.old_run = task.run
        self.old_reset = task.reset
        self.old_children = task.children
        self.execution_count = 0
        
        print("Limit number of executions to: " + str(self.max_executions))
        
    def run(self):
        if self.max_executions != -1 and self.execution_count >= self.max_executions:
            self.execution_count = 0
            
            if self._announce:
                print(self.name + " reached maximum number (" + str(self.max_executions) + ") of executions.")
                
            return TaskStatus.FAILURE
            
        while True:
            self.status = self.old_run()
            
            if self.status == TaskStatus.SUCCESS:
                break
            else:
                return self.status
                
        self.old_reset()
        self.execution_count += 1
    
class ignore_failure(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, task):
        new_name = task.name + "_ignore_failure"
        super(ignore_failure, self).__init__(new_name)

        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            else:
                return self.status
            
class ignore_success(Task):
    """
        Always return FAILURE or RUNNING
    """
    def __init__(self, task):
        new_name = task.name + "_ignore_success"
        super(ignore_success, self).__init__(new_name)

        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.SUCCESS:
                return TaskStatus.FAILURE
            else:
                return self.status

class task_not(Task):
    """
        Turn SUCCESS into FAILURE and vice-versa
    """
    def __init__(self, task):
        new_name = task.name + "_not"
        super(task_not, self).__init__(new_name)

        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            elif self.status == TaskStatus.SUCCESS:
                return TaskStatus.FAILURE
            
            else:
                return self.status

# Alias task_not to invert which seems more intuitive
invert = task_not

class until_fail(Task):
    """
        Execute a task until it fails
    """
    def __init__(self, task):
        new_name = task.name + "_until_fail"
        super(until_fail, self).__init__(new_name)

        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.FAILURE:
                break
            
            else:
                return self.status
            
        return TaskStatus.SUCCESS
    
class always_fail(Task):
    """
        Execute a task but always return FAILTURE
    """
    def __init__(self, task):
        new_name = task.name + "_always_fail"
        super(always_fail, self).__init__(new_name)

        self.old_run = task.run
        
    def run(self):
        while True:    
            self.old_run()
            
            self.status = TaskStatus.FAILURE
            
        return TaskStatus.FAILURE
    
    
def print_tree(tree, indent=0, use_symbols=False):
    """
        Print an ASCII representation of the tree
    """
    if use_symbols:
        if indent == 0:
            print_tree_symbol(tree, indent)
            indent += 1

        for c in tree.children:
            print_tree_symbol(c, indent)

            if c.children != []:
                print_tree(c, indent+1, use_symbols)
    else:
        for c in tree.children:
            print "    " * indent, "-->", c.name
             
            if c.children != []:
                print_tree(c, indent + 1)
                
def print_tree_symbol(c, indent):
    """
        Use ASCII symbols to represent Sequence, Selector, Task, etc.
    """
    if isinstance(c, Selector):
        print "    " * indent, "--?",
    elif isinstance(c, Sequence) or isinstance(c, Iterator):
        print "    " * indent, "-->",
    elif isinstance(c, RandomSequence) or isinstance(c, RandomIterator):
        print "    " * indent, "~~>",
    elif isinstance(c, RandomSelector):
        print "    " * indent, "~~?",
    elif isinstance(c, Loop):
        print "    " * indent, "<->",
    elif isinstance(c, Invert):
        print "    " * indent, "--!",
    else:
        print "    " * indent, "--|",
    
    print c.name
            
def print_phpsyntax_tree(tree):    
    """
        Print an output compatible with ironcreek.net/phpSyntaxTree
    """
    for c in tree.children:
        print "[" + string.replace(c.name, "_", "."),
        if c.children != []:
            print_phpsyntax_tree(c),
        print "]",
    
def print_dot_tree(root, dotfilepath=None):
    """
        Print an output compatible with the DOT synatax and Graphiz
    """
    gr = pgv.AGraph(strict=True, directed=True, rotate='0', bgcolor='white', ordering="out")
    gr.node_attr['fontsize'] = '9'
    gr.node_attr['color'] = 'black'
    
    if dotfilepath is None:
        dotfilepath = os.path.expanduser('~') + '/.ros/tree.dot'
    
    global last_dot_tree
    
    # Add the root node
    gr.add_node(root.name)
    node = gr.get_node(root.name)
    if root.status == TaskStatus.RUNNING:
        node.attr['fillcolor'] = 'yellow'
        node.attr['style'] = 'filled'
        node.attr['border'] = 'bold'
    elif root.status == TaskStatus.SUCCESS:
        node.attr['color'] = 'green'
    elif root.status == TaskStatus.FAILURE:
        node.attr['color'] = 'red'
    else:
        node.attr['color'] = 'black'
                 
    def add_edges(root):
        for c in root.children:
            if isinstance(c, Sequence) or isinstance(c, Iterator) or isinstance(c, RandomSequence) or isinstance(c, RandomIterator):
                gr.add_node(c.name, shape="cds")
            elif isinstance(c, Selector) or isinstance(c, RandomSelector):
                gr.add_node(c.name, shape="diamond")
            elif isinstance(c, ParallelOne) or isinstance(c, ParallelAll):
                gr.add_node(c.name, shape="parallelogram")
            elif isinstance(c, Invert):
                gr.add_node(c.name, shape="house")
                
            gr.add_edge((root.name, c.name))
            node = gr.get_node(c.name)

            if c.status == TaskStatus.RUNNING:
                node.attr['fillcolor'] = 'yellow'
                node.attr['style'] = 'filled'
                node.attr['border'] = 'bold'
            elif c.status == TaskStatus.SUCCESS:
                node.attr['color'] = 'green'
            elif c.status == TaskStatus.FAILURE:
                node.attr['color'] = 'red'
            else:
                node.attr['color'] = 'black'

            if c.children != []:
                add_edges(c)
    
    add_edges(root)
    
    current_dot_tree = gr.string()
        
    if current_dot_tree != last_dot_tree:
        gr.write(dotfilepath)
        last_dot_tree = gr.string()
