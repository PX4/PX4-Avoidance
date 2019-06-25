#!/usr/bin/python

import sys

NORMAL = "normal"
FALLBACK = "fallback"

class Transition:
    def __init__(self, start, edge_name, end, style=NORMAL):
        self.start = start
        self.edge_name = edge_name
        self.end = end
        self.style = style

def extract_first_wrapped(table_string, key):
    try:
        start = table_string.index(key)
    except ValueError:
        return ""
    
    depth = 0
    while depth == 0:
        start += 1
        if table_string[start] == "(":
            depth += 1
    
    end = start + 1
    while depth != 0:
        end += 1
        if table_string[end] == "(":
            depth += 1
        if table_string[end] == ")":
            depth -= 1
            
    state_string = table_string[start:end+1]

    return state_string


def extract_all_wrapped(table_string, key):
    states_strings = []
    end = 0
    while end < len(table_string):
        state_n = extract_first_wrapped(table_string[end:], key)
        if len(state_n) == 0:
            break
        states_strings.append(state_n)
        end += table_string[end:].index(state_n) + len(state_n)
    return states_strings


def extract_transitions(file_string):
    table_string = extract_first_wrapped(file_string, "USM_TABLE")
    state_strings = extract_all_wrapped(table_string, "USM_STATE")
    error_state = table_string.split(",")[1].strip().split("::")[-1]
    transitions = []
    for state_string in state_strings:
        start_state = state_string.split(",")[1].strip().split("::")[-1]
        transition_strings = extract_all_wrapped(state_string, "USM_MAP")
        # print start_state, transition_strings
        state_transitions = []
        error_handled = False
        for transition_string in transition_strings:
            edge_name, end_state = (transition_string.replace("("," ").replace(")"," ").split(","))
            new_transition = Transition(start_state,
                                        edge_name.strip().split("::")[-1],
                                        end_state.strip().split("::")[-1])
            if new_transition.edge_name == "ERROR":
                error_handled = True
            state_transitions.append(new_transition)
        if not error_handled:
            for t in state_transitions:
                if t.start == start_state and t.end == error_state:
                    t.edge_name += "\\nERROR"
                    error_handled = True
                    break
        if not error_handled:
            state_transitions.append(Transition(start_state, "ERROR", error_state, FALLBACK))
        transitions.extend(state_transitions)
    return transitions

def make_dot_file_string(transitions):
    output = []
    output.append("digraph {")
    for t in transitions:
        weight = 1
        if t.style == NORMAL:
            style = "solid"
        elif t.style == FALLBACK:
            style = "dotted"
            weight = 0.1
        else:
            style = "dashed"
        output.append("    \"{start}\" -> \"{end}\" [label=\"{name}\", "
                                                    "style=\"{style}\", "
                                                    "weight={weight}]".format(start=t.start,
                                                                              end=t.end, 
                                                                              name=t.edge_name,
                                                                              style=style,
                                                                              weight=weight))
    output.append("}")
    return "\n".join(output)


def help():
    print("Usage: generate_flow_diagram.py input_file [output_file]")


def main():
    arguments = sys.argv[1:]
    if len(arguments) == 0:
        help()
        exit(1)
    
    filename = arguments[0]
    
    with open(filename, 'r') as file:
        cpp_string = file.read()

    transitions = extract_transitions(cpp_string)
    dot_file_string = make_dot_file_string(transitions)

    if len(arguments) == 2:
        out_filename = arguments[1]
    else:
        out_filename = filename + ".dot"

    with open(out_filename, 'w') as output_file:
        output_file.write(dot_file_string)

if __name__ == "__main__":
    main()
