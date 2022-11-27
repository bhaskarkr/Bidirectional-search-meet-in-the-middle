from random_maze.maze import Maze
def main(w= 20, h= 20, n =1, empty = False):
    
    for i in range(n):
        a = Maze(w, h)
        
        if not empty:
            a.randomize()
        mat = a._to_str_matrix()
        res = ""
        if empty:
            # every cell " " except the border
            for j in range(len(mat)):
                if j==0 or j==len(mat)-1:
                    mat[j] =["%"]*len(mat[j])
                else:
                    mat[j] = ["%"] + [" "]*len(mat[j][1:-1]) + ["%"]
                res += "".join(mat[j]) 
                res += "\n"
        else:
            for row in mat:
                res += "".join(row)
                res += "\n"
        # starting pos
        (x,y) = a._get_random_position(start=True)
        res = a.set_display(y , x , 'P', res)

        #ending pos
        (x,y)= a._get_random_position(start=False)
        res = a.set_display(y , x , '.', res)
        # finish x,y in pacman cooridinates:
        (x,y) = (x,a.height*2-y)
        # To Do: add agent start and end positions
        # save res to lay file
        if empty:
            testfile = open(f"layouts/random/randomEmpty_{i}_finish({x},{y}).lay","w")
        else:
            testfile = open(f"layouts/random/random_{i}_finish({x},{y}).lay","w")
        testfile.write(res)

if __name__ == "__main__":
    # if arg multiple, create multiple mazes
    # if arg -w, set width
    # if arg -h, set height
    import argparse
    parser = argparse.ArgumentParser()
    
    parser.add_argument("-width", "--width", type=int, default=20)
    parser.add_argument("-height", "--height", type=int, default=20)
    parser.add_argument("-n", "--number", type=int, default=1)
    #empty maze
    parser.add_argument("-e", "--empty", action="store_true")
    args = parser.parse_args()
    main(args.width, args.height, args.number, empty=args.empty)