#!/usr/bin/env python3
import sys
import math
import random

from time import time
from operator import itemgetter

from fishing_game_core.game_tree import Node
from fishing_game_core.player_utils import PlayerController
from fishing_game_core.shared import ACTION_TO_STR
from fishing_game_core.shared import TYPE_TO_SCORE

TIME_LIMIT = 55 * 1e-3


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
        self.start_t = 0
        self.max_depth = 0

    def player_loop(self):
        """
        Main loop for the minimax next move search.
        :return:
        """

        # Generate first message (Do not remove this line!)
        first_msg = self.receiver()

        while True:
            msg = self.receiver()

            # Create the root node of the game tree
            node = Node(message=msg, player=PLAYER.A)

            # Possible next moves: "stay", "left", "right", "up", "down"
            best_move = self.search_best_next_move(current_node=node)

            # Execute next action
            self.sender({"action": best_move, "search_time": None})

    def search_best_next_move(self, current_node):
        """
        Use minimax (and extensions) to find best possible next move for player 0 (green boat)
        :param current_node: Initial game tree node
        :type current_node: game_tree.Node
            (see the Node class in game_tree.py for more information!)
        :return: either "stay", "left", "right", "up" or "down"
        :rtype: str
        """

        depth = 0
        moves = []
        flag = True
        self.start_t = time()
        while flag:
            v, m, flag = self.min_max(player=PLAYER.A, node=current_node, alpha=float('-inf'), beta=float('inf'), depth=depth)
            if (m is not None): 
                moves.append((v,m))
            depth = depth+1

        if len(moves):
            best_move = max(moves, key=itemgetter(0))[1]
        else:
            best_move = 0
        
        return ACTION_TO_STR[best_move]

    def min_max(self, player, node, alpha, beta, depth):
        if time() - self.start_t > TIME_LIMIT:
            return self.heuristic(node), node.move, False

        children = node.compute_and_get_children()
        children.sort(key=self.heuristic, reverse=True)

        if len(children) == 0 or depth == 0:
            return self.heuristic(node), node.move, True

        best_move = None
        if player == PLAYER.A:
            v = float('-inf')
            for child in children:
                child_v, child_m, _ = self.min_max(PLAYER.B, child, alpha, beta, depth - 1)
                if child_v > v:
                    v = child_v
                    best_move = child_m

                alpha = max(v, alpha)
                if alpha >= beta:
                    break

            return v, best_move, True
        
        else: # player B
            v = float('inf')
            for child in children:
                child_v, child_m, _ = self.min_max(PLAYER.A, child, alpha, beta, depth - 1)
                if child_v < v:
                    v = child_v
                    best_move = child_m

                beta = min(v, beta)
                if beta <= alpha:
                    break

            return v, best_move, True

    def heuristic(self, node):
        state = node.state
        score = state.get_player_scores()  # (Player.A, Player.B)
        hook = state.get_hook_positions()  # {idx: (x,y) ...}
        fish = state.get_fish_positions()  # {idx: (x,y) ...}
        fish_scores = state.get_fish_scores()

        if (len(fish) == 0):
            return score[PLAYER.A] - score[PLAYER.B]
        
        score_a = score[PLAYER.A]
        score_b = score[PLAYER.B]
        for f in fish:
            distA = self.distance(fish[f], hook[0])
            distB = self.distance(fish[f], hook[1])

            score_a += fish_scores[f] * math.exp(-distA)
            score_b += fish_scores[f] * math.exp(-distB)
        
        return score_a - score_b

    def distance(self, fish, hook):
        dx = abs(fish[0] - hook[0])
        dy = abs(fish[1] - hook[1])
        return dx + dy
