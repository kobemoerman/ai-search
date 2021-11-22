#!/usr/bin/env python3
import random

from time import time
from typing import NamedTuple
from operator import itemgetter

from fishing_game_core.game_tree import Node
from fishing_game_core.player_utils import PlayerController
from fishing_game_core.shared import ACTION_TO_STR
from fishing_game_core.shared import TYPE_TO_SCORE

TIME_LIMIT  = 55 * 1e-3
DEPTH_LIMIT = 8

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
        self.avg_depth = 0

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

        flag = False
        a = float('-inf')
        b = float('inf')
        self.start_t = time()
        self.encoded = dict()

        moves = []
        for depth in range(1, DEPTH_LIMIT):
            v, m, flag = self.alpha_beta(player=PLAYER.A, node=current_node, alpha=a,  beta=b, depth=depth)

            if m is not None:
                moves.append((v, m))

            if flag:
                break

        if len(moves):
            best_move = max(moves, key=itemgetter(0))[1]
        else:
            best_move = random.randrange(5)

        return ACTION_TO_STR[best_move]

    def alpha_beta(self, player, node, alpha, beta, depth):
        node = self.repeated_state(node, player)
        
        if time() - self.start_t > TIME_LIMIT:
            v = self.heuristic(node)
            return v, node.move, True

        children = node.compute_and_get_children()
        children.sort(key=self.heuristic, reverse=(player==PLAYER.A))

        if depth == 0 or len(children) == 0:
            v = self.heuristic(node)
            return v, node.move, False
        
        if player == PLAYER.A: # PLAYER A
            v = float('-inf')
            for child in children:
                child_v, _, _ = self.alpha_beta(PLAYER.B, child, alpha, beta, depth-1)
                if child_v > v:
                    v, move = child_v, child.move
                alpha = max(v, alpha)
                if alpha >= beta:
                    break
            return v, move, False

        else: # PLAYER B
            v = float('inf')
            for child in children:
                child_v, _, _ = self.alpha_beta(PLAYER.A, child, alpha, beta, depth-1)
                if child_v < v:
                    v, move = child_v, child.move
                beta = min(v, beta)
                if alpha >= beta:
                    break
            return v, move, False

    def repeated_state(self, node, player):
        key = self.hash_state(node.state, player)
        
        if self.encoded.get(key) is None:
            self.encoded[key] = node

        return self.encoded.get(key)


    def heuristic(self, node):
        state = node.state
        score = state.get_player_scores()  # (Player.A, Player.B)
        hook = state.get_hook_positions()  # {idx: (x,y) ...}
        fish = state.get_fish_positions()  # {idx: (x,y) ...}
        fish_scores = state.get_fish_scores()

        score_a = score[PLAYER.A]
        score_b = score[PLAYER.B]

        best_fish_score = 0
        for f in fish:
            dist_a = self.distance(fish[f], hook[0], hook[1])
            dist_b = self.distance(fish[f], hook[1], hook[0])

            best_fish_score += fish_scores[f] / (dist_a + .1)
            best_fish_score -= fish_scores[f] / (dist_b + .1)
            # ToDo - improve
            # f_a = max(0, fish_scores[f] / (dist_a + 1))
            # f_b = min(0, fish_scores[f] / (dist_b + 1))
            # best_fish_score = max(f_a + f_b, best_fish_score)

        return (score_a - score_b) + best_fish_score

    def distance(self, fish, hook, opponent_hook):
        dy = abs(fish[1] - hook[1])

        if hook[0] < opponent_hook[0] < fish[0]:
            dx = hook[0] + (20 - fish[0])
        elif hook[0] > opponent_hook[0] > fish[0]:
            dx = (20 - hook[0]) + fish[0]
        else:
            dx = abs(fish[0] - hook[0])

        return dx + dy
        

    def hash_state(self, state, player):
        fish_hash = {}
        
        hook  = state.get_hook_positions()  # {idx: (x,y) ...}
        fish  = state.get_fish_positions()  # {idx: (x,y) ...}
        fish_score = state.get_fish_scores()

        for pos, value in zip(fish, fish_score):
            fish_hash.update({fish.get(pos):fish_score.get(value)})

        score = state.player_scores[0] - state.player_scores[1]

        return str(player)+str(score)+str(hook)+str(fish_hash)
        