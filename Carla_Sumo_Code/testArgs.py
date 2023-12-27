import argparse

class Error(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(message)        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Description of your script.")

    # Add command-line arguments
    parser.add_argument("--attackID", type=int, help="1-Sybil Attack, 2- Bias Injection Attack", default=0)
    parser.add_argument("--spoofedCAVCount", type=int, help="Total number of spoofed CAVs", default= 0)
    parser.add_argument("--arrivalTime", type=int, help="Arrival time of spoofed CAVs", default= [])
    parser.add_argument("--maximumBias", type=float, help="", default= 0)

    # Parse the command-line arguments
    args = parser.parse_args()

    try:
        if len(args.arrivalTime) != args.spoofedCAVCount:
            raise ValueError("Maximum number of spoofed CAVs should be less than the total number of CAVs") 
    except Error as e:
        print(e)

    try:
        if args.attackID < 0 or args.attackID>2:
            raise ValueError("Attack if is not valid") 
    except Error as e:
        print(e)

    try:
        if args.maximumBias:
            raise ValueError("Maximum bias in velocity and position is 2m/s and 2 m.") 
    except Error as e:
        print(e)

