# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from enum import Enum
import math
import argparse
import sys

class Player(Enum):
    COMPUTER = 1
    HUMAN = 2

class GameState:
    def __init__(self, red_marbles, blue_marbles, turn):
        self.red_marbles = red_marbles
        self.blue_marbles = blue_marbles
        self.turn = turn

def game_over(state):
    return state.red_marbles == 0 or state.blue_marbles == 0

def evaluate_state(state):
    if state.red_marbles > state.blue_marbles:
        return 1
    elif state.blue_marbles > state.red_marbles:
        return -1
    else:
        return 0

def max_value(state, alpha, beta, depth_limit, current_depth):
    if game_over(state) or current_depth >= depth_limit:
        return evaluate_state(state)
    v = -math.inf
    for i in range(1, state.red_marbles + 1 if state.turn == Player.COMPUTER else state.blue_marbles + 1):
        if state.turn == Player.COMPUTER:
            new_state = GameState(state.red_marbles - i, state.blue_marbles, Player.HUMAN)
        else:
            new_state = GameState(state.red_marbles, state.blue_marbles - i, Player.COMPUTER)
        v = max(v, min_value(new_state, alpha, beta, depth_limit, current_depth + 1))
        if v >= beta:
            return v
        alpha = max(alpha, v)
    return v

def min_value(state, alpha, beta, depth_limit, current_depth):
    if game_over(state) or current_depth >= depth_limit:
        return evaluate_state(state)
    v = math.inf
    for i in range(1, state.red_marbles + 1 if state.turn == Player.COMPUTER else state.blue_marbles + 1):
        if state.turn == Player.COMPUTER:
            new_state = GameState(state.red_marbles - i, state.blue_marbles, Player.HUMAN)
        else:
            new_state = GameState(state.red_marbles, state.blue_marbles - i, Player.COMPUTER)
        v = min(v, max_value(new_state, alpha, beta, depth_limit, current_depth + 1))
        if v <= alpha:
            return v
        beta = min(beta, v)
    return v

def computer_player(state, max_depth):
    best_move = None
    best_score = -math.inf
    current_depth = 1
    while current_depth <= max_depth:
        for color in ['red', 'blue']:
            for i in range(1, state.red_marbles + 1 if color == 'red' else state.blue_marbles + 1):
                if color == 'red':
                    new_state = GameState(state.red_marbles - i, state.blue_marbles, Player.HUMAN)
                else:
                    new_state = GameState(state.red_marbles, state.blue_marbles - i, Player.HUMAN)
                score = min_value(new_state, -math.inf, math.inf, current_depth, 1)
                if score > best_score:
                    best_score = score
                    best_move = (color, i)
        current_depth += 1
    return best_move

def human_player(state):
    while True:
        pile = input("Choose a pile to remove a marble from (red or blue): ")
        while pile != 'red' and pile != 'blue':
            pile = input("Invalid choice. Choose a pile to remove a marble from (red or blue): ")

        max_marbles = state.red_marbles if pile == 'red' else state.blue_marbles
        if max_marbles == 0:
            print("There are no marbles left in that pile. Choose another pile.")
            continue

        num_marbles = input(f"Choose the number of marbles to remove from the {pile} pile (1 to {max_marbles}): ")
        while not num_marbles.isdigit() or int(num_marbles) < 1 or int(num_marbles) > max_marbles:
            num_marbles = input(f"Invalid choice. Choose the number of marbles to remove from the {pile} pile (1 to {max_marbles}): ")

        return pile, int(num_marbles)

def calculate_score(red_marbles, blue_marbles):
    return (2 * red_marbles) + (3 * blue_marbles)

def determine_winner(red_marbles, blue_marbles):
    if red_marbles == 0 and blue_marbles == 0:
        return "It's a Tie"
    elif red_marbles == 0:
        return "Computer Wins"
    else:
        return "Human Wins"

def parse_args():
    parser = argparse.ArgumentParser(description='Marble game command line interface.')
    parser.add_argument('--red', type=int, default=10, help='Number of red marbles')
    parser.add_argument('--blue', type=int, default=10, help='Number of blue marbles')
    parser.add_argument('--first', choices=['human', 'computer'], default='human', help='Which player goes first')
    parser.add_argument('--depth-limited', action='store_const', const=True, default=False, help='Use depth-limited search')
    return parser.parse_args()


def game_loop(red_marbles, blue_marbles, first_player, use_depth_limited):
    state = GameState(red_marbles, blue_marbles, Player[first_player.upper()])
    while state.red_marbles > 0 or state.blue_marbles > 0:
        if state.turn == Player.COMPUTER:
            if use_depth_limited:
                depth_limit = None
                while depth_limit is None or not isinstance(depth_limit, int) or int(depth_limit) < 1:
                    depth_limit = input("Enter the depth limit for the computer's search: ")
                    if depth_limit.lower() == 'q':
                        return
                    depth_limit = int(depth_limit)
            else:
                depth_limit = None

            pile, num_marbles = computer_player(state, depth_limit)
            print(f"The computer removes {num_marbles} marble(s) from the {pile} pile.")
            if pile == 'red':
                state.red_marbles -= num_marbles
            else:
                state.blue_marbles -= num_marbles
        else:
            pile, num_marbles = human_player(state)
            print(f"You remove {num_marbles} marble(s) from the {pile} pile.")
            if pile == 'red':
                state.red_marbles -= num_marbles
            else:
                state.blue_marbles -= num_marbles

        print(f"There are {state.red_marbles} red marble(s) and {state.blue_marbles} blue marble(s) left.")

        if state.red_marbles == 0:
            print("\nRed pile is empty.")
            state.blue_marbles += state.red_marbles
            break
        elif state.blue_marbles == 0:
            print("\nBlue pile is empty.")
            state.red_marbles += state.blue_marbles
            break

        state.turn = Player.COMPUTER if state.turn == Player.HUMAN else Player.HUMAN

    final_score = calculate_score(state.red_marbles, state.blue_marbles)
    print(f"\nRed marbles: {state.red_marbles}\nBlue marbles: {state.blue_marbles}\n")
    print(f"The final score is: {final_score}\n")
    winner = determine_winner(state.red_marbles, state.blue_marbles)
    print(f"Game status: {winner}\n")

def main():
    if len(sys.argv) == 5:
        red_marbles = int(sys.argv[1])
        blue_marbles = int(sys.argv[2])
        first_player = sys.argv[3]
        use_depth_limited = True if sys.argv[4].lower() == 'true' else False
        game_loop(red_marbles, blue_marbles, first_player, use_depth_limited)
    else:
        print("Invalid arguments. Usage: python red_blue_nim.py <red_marbles> <blue_marbles> <first_player> <use_depth_limited>")


if __name__ == '__main__':
    main()