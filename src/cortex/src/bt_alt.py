#!/usr/bin/env python

from owyl import blackboard
from owyl import taskmethod, visit, succeed, fail
from owyl import sequence, selector, parallel, PARALLEL_SUCCESS
from owyl.decorators import repeatUntilFail, limit, repeatAlways

class btree():               #??? input and output ???
    def __init__(self, blackboard):
        self.bb = blackboard
        self.tree = self.buildTree()

    def buildTree(self):
        '''as simple as nesting thr behavior constructor calls'''
        tree = parallel( limit( repeatAlways(self.checkMyBody(), debug=True), limit_period = 1),    #*children, **kwargs
                                   ### Look & See
                                   ############################
                                    repeatAlways(sequence(self.seeSomethingNew(),               #*children, **kwargs. run until fails
                                                                                self.headtrack(),
                                                                                 self.recogThat()
                                                                                 ),
                                                              ),
                                   self.sayRecog(),

                                   ### Mutter
                                   #############################
                                   self.mutter( ),

                                   ### Chatterbox
                                   #############################
                                   self.chat( ),

                                   policy=PARALLEL_SUCCESS.REQUIRE.ALL
                         )
        return visit(tree, blackboard = self.bb)

    def xxx(self):
        pass

if __name__ == '__main__':
    bt = btree()
    
                         
