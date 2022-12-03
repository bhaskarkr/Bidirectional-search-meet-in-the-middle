# Meet in the Middle (MM)
Note: This markdown file is compiled into PDF, use the PDF for a better reading experience.
## Code Dependencies
This Project is written in Python 3.7 and has dependencies on the following libraries:
* [scipy](https://www.scipy.org/)
* [Pandas](https://pandas.pydata.org/)


## To Reproduce Results 
`python pacman.py -p SearchAgent --randomtest 1 --frameTime 0 --quietTextGraphics` <br>

This will run every algorithm for every layout in `layouts/random`. Different sets of randomized layouts are provided in `layouts/randomLinear`, `layout/randomMaze`, and `layout/randomOpen`. These sets can be copied into layouts/random for testing. Additionally, a one way T-test is conducted between every algorithm pair on the nodes expanded. The results will be generated in the `TTest_results.csv` file, while the nodes expanded for each run will be saved to `stats.csv` file, in the root folder. **Important Note:** Make sure you copy files from `layouts/randomLinear`, `layout/randomMaze`, or `layout/randomOpen` into `layouts/random` each time you run. **Make sure you delete the previous mazes in `layouts/random`**<br> 


## To generate random layouts
To generate random layouts: <br>
`python createRandom.py -width 15 -height 15 -n 1 ` <br>
width- width of the maze <br>
height- height of the maze <br>
n- number of mazes to generate <br>
To generate random layouts without walls: <br>
`python createRandom.py -width 15 -height 15 -n 1 -e` <br>
This generates the layouts in the `layouts/random` directory. <br>

Examples used to generate results layout folders:<br>
open layouts - `python createRandom.py -width 15 -height 15 -n 50 -e` <br>
maze layouts - `python createRandom.py -width 15 -height 15 -n 50` <br>
linear layouts - `python createRandom.py -width 2 -height 15 -n 50` <br>


## To run MM on a single layout
`python pacman.py -l bigMaze -p SearchAgent -a fn=mm,heuristic=terminalNodeManhattanHeuristic -z 0.5` 
The options for heuristics are: <br>
* heuristic= mmNullHeuristic 
* heuristic= terminalNodeManhattanHeuristic
* heuristic= oppositeDirectionlastVisitedEuclideanHeuristic
* heuristic= terminalNodeEuclideanHeuristic 
* heuristic= oppositeDirectionlastVisitedManhattanHeuristic 


## To run MM on a randomly generated layout with a custom goal state
`python pacman.py -l random_0_finish(20,2) -p SearchAgent -a fn=mm,heuristic=terminalNodeManhattanHeuristic --x 20 --y 2` <br>
Here x and y are the coordinates of the goal state, which must be the same as specified in the layout file name, for example for random_0_finish(x,y): x and y are the coordinates. It should also be noted that the random layout must be in the `/layouts` directory as opposed to the `/layouts/random` directory. <br>


## To generate the two heatmap examples comparing BFS expansion to MM expansion
`python pacman.py -l openMaze -p SearchAgent -a fn=bfs` <br>
`python pacman.py -l openMaze -p SearchAgent -a fn=mm` <br>


