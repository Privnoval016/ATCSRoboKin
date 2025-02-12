# A parser for the instruction files used by the robot
# The format of the instruction files can be seen in instructions.txt

def parse_instruction_file(file_name):

    def is_true(value):
        return (value.upper() == "TRUE"
                or value == "1" or value.upper() == "CCW")

    def is_false(value):
        return (value.upper() == "FALSE"
                or value == "0" or value.upper() == "CW")

    operations = []

    valid_chars = \
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._$-"

    default_params = {
        "LIN_STEP": 0.1,
        "ANG_STEP": 10,
        "TIME_SCALE": 0.1,
        "AUTO_TURN": False,
        "X_EXTENT": 0.105,
        "Y_EXTENT": 0.150,
        "PLOT_VEL": True,
        "PLOT_DIR": True,
        "LIN_VEL": 1,
        "ANG_VEL": 90
    }

    rot_type = ["MATCH_VELOCITY", 0]

    operation_dict = {
        "START": 0,
        "POLYGON": 1,
        "ARC": 2,
        "SEMICIRCLE": 3,
        "LINE": 4,
        "POINT_TURN": 5,
    }

    with open(file_name) as f:
        lines = f.readlines()

    command = ""

    for line in lines:
        line = line.lstrip()
        if line == "" or line[0] == "#":
            continue

        line = "".join([char if char in valid_chars else " " for char in line])
        instruction = line.split()

        instruction = [token for token in instruction if token[0] != "_"]

        if instruction[0][0] == "$":
            command = instruction[0].upper()
        else:
            instruction.insert(0, command)

        if len(instruction) < 2:
            continue

        if (any(c.isalpha() for c in instruction[-1])
                and not is_true(instruction[-1])
                and not is_false(instruction[-1])):
            if instruction[-1][-1].isnumeric():
                num = ""
                for char in instruction[-1][::-1]:
                    if (char.isnumeric() or char == "."
                            or char == "-" or char == "+"):
                        num = char + num
                    else:
                        break
                rot_type = [instruction[-1][:-len(num)].replace(
                    "_", ""), float(num)]
            else:
                rot_type = [instruction[-1].replace("_", ""), 0]

            instruction = instruction[:-1]

        match command:
            case "$D":
                default_params[instruction[1].upper()] = float(instruction[2])

            case "$S":
                params = [(float(instruction[i]), float(instruction[i+1]))
                          for i in range(2, len(instruction), 2)]
                op = operation_dict[instruction[1].upper()]
                operations.append((op, params, rot_type))

            case "$A":
                if instruction[1].isdigit():
                    op = int(instruction[1])
                else:
                    op = operation_dict[instruction[1].upper()]

                # combine pairs of arguments into a tuple
                if len(instruction) % 2 == 1:
                    params = [(float(instruction[i]), float(instruction[i+1]))
                              for i in range(2, len(instruction) - 1, 2)]
                    if is_true(instruction[-1]):
                        params.append(True)
                    elif is_false(instruction[-1]):
                        params.append(False)
                else:
                    params = [(float(instruction[i]), float(instruction[i+1]))
                              for i in range(2, len(instruction), 2)]

                operations.append((op, params, rot_type))

            case "$R":
                if instruction[1].isdigit():
                    op = int(instruction[1])
                else:
                    op = operation_dict[instruction[1].upper()]

                # combine pairs of arguments into a tuple
                if len(instruction) % 2 == 1:
                    params = [(float(instruction[i]), float(instruction[i+1]))
                              for i in range(2, len(instruction) - 1, 2)]
                    if is_true(instruction[-1]):
                        params.append(True)
                    elif is_false(instruction[-1]):
                        params.append(False)
                else:
                    params = [(float(instruction[i]), float(instruction[i+1]))
                              for i in range(2, len(instruction), 2)]

                params.insert(0, 0)

                operations.append((op, params, rot_type))

    return operations, default_params
