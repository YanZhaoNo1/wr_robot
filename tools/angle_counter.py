#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-
import math

# define color
COLORS = {
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
    "white": "\033[37m",
    "reset": "\033[0m"
}


def print_color(text, color='reset' ,end="\n"):
    # print text in color
    print(COLORS[color] + text + COLORS["reset"], end=end)

def input_color(text, color):
    # input text in color
    return input(COLORS[color] + text + COLORS["reset"])

# quaternion z and w --> Euler z angle
def quaternion_to_z_angle(q):
    qz = q[0]
    qw = q[1]
    theta = 2 * math.atan2(qz, qw) * 180 / math.pi
    if theta < 0:
        theta += 360
    return theta


# Euler z angle --> quaternion z and w
def z_angle_to_quaternion(theta):
    theta = theta % 360
    qw = math.cos(theta/2 * math.pi/180)
    qz = math.sin(theta/2 * math.pi/180)
    q = [0, 0, qz, qw]
    return q


def main():
    while True:
        print_color("[q]",'cyan',"\t\t")
        print_color("quaternion_to_angle",'blue')
        print_color("[a]",'cyan',"\t\t")
        print_color("z_angle_to_quaternion",'blue')
        print_color("[(others)]",'cyan',"\t")
        mode = input_color("exit\n",'blue')
        if mode == "q":
            while True:
                try:
                    q = [0, 0]
                    print_color("input quaternion:",'yellow',"\t")
                    q[0], q[1] = (input_color(
                        "(example: z w )\n",'green').split())
                    q[0] = float(q[0])
                    q[1] = float(q[1])
                    a = quaternion_to_z_angle(q)
                except ValueError:
                    print_color("go back to mode select",'red')
                    break
                print_color("z_angle is %.3f" % a,'magenta')
        elif mode == "a":
            while True:
                try:
                    print_color("input z_angle:",'yellow',"\t")
                    a = input_color("(example: theta)\n",'green')
                    a = float(a)
                    q = z_angle_to_quaternion(a)
                except ValueError:
                    print_color("go back to mode select",'red')
                    break
                print_color(
                    "quaternion is [0.000, 0.000, %.3f, %.3f]" % (q[2], q[3]),'magenta')
        else:
            print_color("exit",'red')
            return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
