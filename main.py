from spine import Namespace, Publisher, Subscriber
from mad import MadType
from robot import Robot

def main():

    ns = Namespace("rime", "ppap")
    input = Subscriber(ns, "goal", tuple[tuple[MadType.float64, 4], 4])
    output = Publisher(ns, "joints", tuple[MadType.float64, 6])

    r1 = Robot("r1", input, output)

    r1.run()


main()