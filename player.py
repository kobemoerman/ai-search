#!/usr/bin/env python3
import sys
import math
import random

from time import time

from fishing_game_core.game_tree import Node
from fishing_game_core.player_utils import PlayerController
from fishing_game_core.shared import ACTION_TO_STR
from fishing_game_core.shared import TYPE_TO_SCORE

TIME_LIMIT = 75*1e-2

class PLAYER:
    A = 0
    B = 1

class PlayerControllerHuman(PlayerController):
    def player_loop(self):
        """
        Function that generates the loop of the game. In each iteration
        the human plays through the keyboard and send
        this to the game through the sender. Then it receives an
        update of the game through receiver, with this it computes the
        next movement.
        :return:
        """

        while True:
            # send message to game that you are ready
            msg = self.receiver()
            if msg["game_over"]:
                return


class PlayerControllerMinimax(PlayerController):

    def __init__(self):
        super(PlayerControllerMinimax, self).__init__()

    def player_loop(self):
        """
        Main loop for the minimax next move search.
        :return:
        """

        # Generate first message (Do not remove this line!)
        first_msg = self.receiver()

        while True:
            msg = self.receiver()
            self.start_t = time()

            # Create the root node of the game tree
            node = Node(message=msg, player=PLAYER.A)

            # Possible next moves: "stay", "left", "right", "up", "down"
            best_move = self.search_best_next_move(current_node=node)

            # Execute next action
            self.sender({"action": best_move, "search_time": None})

    def search_best_next_move(self, current_node):
        """
        Use minimax (and extensions) to find best possible next move for player 0 (green boat)
        :param initial_tree_node: Initial game tree node
        :type initial_tree_node: game_tree.Node
            (see the Node class in game_tree.py for more information!)
        :return: either "stay", "left", "right", "up" or "down"
        :rtype: str
        """

        children = current_node.compute_and_get_children()
        eval = []

        for child in children:
            eval.append(self.min_max(player=PLAYER.B, node=child))
        
        best = eval.index(max(eval))

        return ACTION_TO_STR[children[best].move]
        

    def min_max(self, player, node):
        if time() - self.start_t > 0.05:
            return self.heuristic(node)

        children = node.compute_and_get_children()
        children.sort(key=self.heuristic, reverse = True)

        if (len(children) == 0):
            return self.heuristic(node)

        if player == PLAYER.A:
            c_eval = float('-inf')
            for child in children:
                v = self.min_max(player=PLAYER.B, node=child)
                c_eval = max(v, c_eval)

        else: # player B
            c_eval = float('inf')
            for child in children:
                v = self.min_max(player=PLAYER.A, node=child)
                c_eval = min(v, c_eval)

        return c_eval

    def heuristic(self, node):
        dist = sys.maxsize
        state = node.state
        score = state.get_player_scores() # [Player.A, Player.B]
        hook  = state.get_hook_positions() # {idx: (x,y) ...}
        fish  = state.get_fish_positions() # {idx: (x,y) ...}
        
        type_A = state.get_caught()[0]
        score_A = 0

        if type_A == None:
            for f in fish:
                if state.get_fish_scores()[f] > 0:
                    dist = min(dist, self.distance(fish[f], hook[0]))
        else:
            score_A = TYPE_TO_SCORE[type_A]

        return score[0] - (score_A + score[1] + 0.2*dist)

    def distance(self, fish, hook):
        dx = (hook[0]-fish[0])**2
        dy = (hook[1]-fish[1])**2
        return math.sqrt(dx+dy)

