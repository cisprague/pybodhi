#!/usr/bin/python3

from pybodhi import Tree, Sequence, Fallback

'''
Behaviour tree for SAM demo.

1) if I need to abort:
    abort!
2) if I don't need to abort:
    check if all depths visted
3) if not all depths visted:
    check if we're at the target depth
4) if we're not at the target depth:
    go to the target depth
5) if we're at the target depth:
    update the target depth
6) if all depths visted:
    set need to abort to true
7) go to surface!

Read from top to bottom

         /-_not_need_to_abort
      /?|
     |   \-_go_to_surface 
-Ø /→|
     |   /-_all_depths_visited (if True, mark abort to True)
     |  |
      \?|      /-_at_target_depth
        |   /?|
         \→|   \-_go_to_target_depth
           |
            \-_update_target_depth
'''

class SAM(Tree):

    def __init__(self):

        # safety sub tree
        safety = Fallback([
            self._not_need_to_abort,
            self._go_to_surface
        ])
        
        # depth following sub tree
        depth = Fallback([
            self._at_target_depth,
            self._go_to_target_depth
        ])
        depth = Sequence([
            depth,
            self._update_target_depth
        ])
        depth = Fallback([
            self._all_depths_visited,
            depth
        ])

        # assemble whole tree structure
        tree = Sequence([
            safety,
            depth
        ])

        # become the tree
        Tree.__init__(self, tree)

    # safety
    def _not_need_to_abort(self):
        
        # determine whether
        # we need to abort

        pass

    def _go_to_surface(self):

        # go to surface action

        pass

    # depth following
    def _all_depths_visited(self):

        # determine whether all depths
        # have been visted

        pass

    def _at_target_depth(self):

        # determine whether we're
        # at the target depth in
        # the sin sequence

        pass

    def _go_to_target_depth(self):

        # action to go to
        # target depth

        pass 

    def _update_target_depth(self):

        # change the target depth
        # after previous target depth
        # was achieved

        pass
    
if __name__ == "__main__":

    import ete3

    # instantiate tree
    t = SAM().newick()
    t = ete3.Tree(t, format=1)
    print(t.get_ascii(show_internal=True))
    

