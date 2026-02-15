#!/usr/bin/env python3
# tools/state_to_dot.py - Generate Graphviz DOT from FlightDirector states

import sys
import json  # Assuming states exported as JSON; adjust as needed

def state_to_dot(states):
    dot = "digraph MissionStates {\n"
    dot += "    rankdir=LR;\n"  # Horizontal layout
    for state, transitions in states.items():
        for target, condition in transitions.items():
            dot += f'    "{state}" -> "{target}" [label="{condition}"];\n'
    dot += "}\n"
    return dot

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: state_to_dot.py <states.json>")
        sys.exit(1)
    with open(sys.argv[1], 'r') as f:
        states = json.load(f)
    print(state_to_dot(states))
# Run: python tools/state_to_dot.py states.json > states.dot; dot -Tsvg states.dot -o docs/states.svg
