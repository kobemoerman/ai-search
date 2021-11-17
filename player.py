#!/usr/bin/env python3
import sys
import math
import random

from time import time

from fishing_game_core.game_tree import Node
from fishing_game_core.player_utils import PlayerController
from fishing_game_core.shared import ACTION_TO_STR
from fishing_game_core.shared import TYPE_TO_SCORE

TIME_LIMIT = 75 * 1e-3


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

        self.start_t = time()
        # best_move = 0
        self.max_depth = 10
        # while time() - self.start_t <= 0.05:
        v, best_move = self.min_max(player=PLAYER.A, node=current_node, alpha=float('-inf'), beta=float('inf'))
        # self.max_depth += 1

        print(v)
        return ACTION_TO_STR[best_move]

    def min_max(self, player, node, alpha, beta, depth=0):
        if time() - self.start_t > 0.05 or depth == self.max_depth:
            return self.heuristic(node), node.move

        children = node.compute_and_get_children()
        children.sort(key=self.heuristic, reverse=True)

        if len(children) == 0:
            scores = node.state.get_player_scores()
            # return scores[0] - scores[1], node.move
            return self.heuristic(node), node.move

        best_move = None
        if player == PLAYER.A:
            v = float('-inf')
            for child in children:
                child_v, child_move = self.min_max(PLAYER.B, child, alpha, beta, depth + 1)
                if child_v > v:
                    v = child_v
                    best_move = child_move

                alpha = max(v, alpha)
                if v >= beta:
                    break

            return v, best_move

        v = float('inf')
        for child in children:
            child_v, child_move = self.min_max(PLAYER.A, child, alpha, beta, depth + 1)
            if child_v < v:
                v = child_v
                best_move = child_move

            beta = min(v, beta)
            if v <= alpha:
                break

        return v, best_move

    def heuristic(self, node):
        state = node.state
        score = state.get_player_scores()  # [Player.A, Player.B]
        hook = state.get_hook_positions()  # {idx: (x,y) ...}
        fish = state.get_fish_positions()  # {idx: (x,y) ...}
        fish_scores = state.get_fish_scores()

        caught = state.get_caught()
        # score_A = 0

        scr = 0
        score_a = 0
        score_b = 0 if caught[1] is None else TYPE_TO_SCORE[caught[1]]

        if caught[0] is None:
            scr = float('-inf')
            for f in fish:
                if fish_scores[f] > 0:
                    a_dist = self.distance(fish[f], hook[0])
                    b_dist = self.distance(fish[f], hook[1]) if caught[1] is None else 0
                    # b_dist = self.distance(fish[f], hook[1])
                    scr = max(scr, (b_dist - a_dist) + (fish_scores[f] * 0.1))
        else:
            score_a = TYPE_TO_SCORE[caught[0]]

        return score[0] - score[1] + scr * 2 + score_a - score_b
        # return score[0] - (score_A + score[1] + 0.2 * dist)

    def distance(self, fish, hook):
        # dx = (hook[0] - fish[0]) ** 2
        # dy = (hook[1] - fish[1]) ** 2
        dx = (hook[0] - fish[0])
        dy = (hook[1] - fish[1])
        return abs(dx + dy)
