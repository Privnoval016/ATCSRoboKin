class Color:
    def __init__(self, r: int, g: int, b: int, a: int = 255):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    def __repr__(self):
        return f"Color({self.r}, {self.g}, {self.b}, {self.a})"

    def __add__(self, other):
        if isinstance(other, Color):
            return Color(
                min(self.r + other.r, 255),
                min(self.g + other.g, 255),
                min(self.b + other.b, 255),
                min(self.a + other.a, 255)
            )
        else:
            raise TypeError("Unsupported operand type(s) for +: 'Color' and '{}'".format(type(other)))

    def __sub__(self, other):
        if isinstance(other, Color):
            return Color(
                max(self.r - other.r, 0),
                max(self.g - other.g, 0),
                max(self.b - other.b, 0),
                max(self.a - other.a, 0)
            )
        else:
            raise TypeError("Unsupported operand type(s) for -: 'Color' and '{}'".format(type(other)))


    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Color(
                min(int(self.r * scalar), 255),
                min(int(self.g * scalar), 255),
                min(int(self.b * scalar), 255),
                min(int(self.a * scalar), 255)
            )
        else:
            raise TypeError("Unsupported operand type(s) for *: 'Color' and '{}'".format(type(scalar)))

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            if scalar == 0:
                raise ZeroDivisionError("division by zero")
            return Color(
                max(int(self.r / scalar), 0),
                max(int(self.g / scalar), 0),
                max(int(self.b / scalar), 0),
                max(int(self.a / scalar), 0)
            )
        else:
            raise TypeError("Unsupported operand type(s) for /: 'Color' and '{}'".format(type(scalar)))


    def __eq__(self, other):
        if isinstance(other, Color):
            return (self.r == other.r and
                    self.g == other.g and
                    self.b == other.b and
                    self.a == other.a)
        else:
            raise TypeError("Unsupported operand type(s) for ==: 'Color' and '{}'".format(type(other)))

    def __ne__(self, other):
        if isinstance(other, Color):
            return not (self.r == other.r and
                        self.g == other.g and
                        self.b == other.b and
                        self.a == other.a)
        else:
            raise TypeError("Unsupported operand type(s) for !=: 'Color' and '{}'".format(type(other)))


    def __iter__(self):
        return iter((self.r, self.g, self.b, self.a))


    def to_tuple(self):
        return (self.r, self.g, self.b, self.a)


    def to_hex(self):
        return "#{:02x}{:02x}{:02x}".format(self.r, self.g, self.b,)


    @staticmethod
    def from_hex(hex_str: str):
        hex_str = hex_str.lstrip('#')
        r = int(hex_str[0:2], 16)
        g = int(hex_str[2:4], 16)
        b = int(hex_str[4:6], 16)
        return Color(r, g, b)


    @staticmethod
    def RED():
        return Color(255, 0, 0)


    @staticmethod
    def GREEN():
        return Color(0, 255, 0)


    @staticmethod
    def BLUE():
        return Color(0, 0, 255)


    @staticmethod
    def WHITE():
        return Color(255, 255, 255)


    @staticmethod
    def BLACK():
        return Color(0, 0, 0)


    @staticmethod
    def YELLOW():
        return Color(255, 255, 0)


    @staticmethod
    def CYAN():
        return Color(0, 255, 255)


    @staticmethod
    def MAGENTA():
        return Color(255, 0, 255)


    @staticmethod
    def GRAY():
        return Color(128, 128, 128)


    @staticmethod
    def LIGHT_GRAY():
        return Color(192, 192, 192)


    @staticmethod
    def DARK_GRAY():
        return Color(64, 64, 64)


    @staticmethod
    def TRANSPARENT():
        return Color(0, 0, 0, 0)
