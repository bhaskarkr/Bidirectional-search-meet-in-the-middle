# Meet in the Middle (MM)
Note: This markdown file is compiled into PDF, use the PDF for a better reading experience.
## Code Dependencies
This Project is written in Python 3.7 and has dependencies on the following libraries:
* [scipy](https://www.scipy.org/)
* [Pandas](https://pandas.pydata.org/)

To install dependencies run the following command: <br>
`pip install -r requirements.txt` <br>

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

**Note:** We are using same seed for the random function, so for a single command it will always same set of layout given the args are same.


## To Reproduce Results 
**Important Note:** Layout folders with generated layours are provided for results reproducibility. Make sure you copy files from `layouts/randomLinear`, `layout/randomMaze`, or `layout/randomOpen` into `layouts/random` each time you run. **Make sure you delete the previous mazes in `layouts/random` and that the folder `layouts/random` exists**<br>

To Reproduce Maze-like layout results: <br>
delete layouts in `layouts/random` <br>
copy layouts from `layout/randomMaze` to `layouts/random` <br>
then run the following command `python pacman.py -p SearchAgent --randomtest 1 --frameTime 0 --quietTextGraphics` <br>

To Reproduce Linear layout results: <br>
delete layouts in `layouts/random` <br>
copy layouts from `layout/randomLinear` to `layouts/random` <br>
then run the following command `python pacman.py -p SearchAgent --randomtest 1 --frameTime 0 --quietTextGraphics` <br>

To Reproduce Open layout results: <br>
delete layouts in `layouts/random` <br>
copy layouts from `layout/randomOpen` to `layouts/random` <br>
then run the following command `python pacman.py -p SearchAgent --randomtest 1 --frameTime 0 --quietTextGraphics` <br>

The python command will run every algorithm for every layout in `layouts/random`. Different sets of randomized layouts are provided in `layouts/randomLinear`, `layout/randomMaze`, and `layout/randomOpen`. These sets can be copied into layouts/random for testing. Additionally, a one way T-test is conducted between every algorithm pair on the nodes expanded. The results will be generated in the `TTest_results.csv` file, while the nodes expanded for each run will be saved to `stats.csv` file, in the root folder. <br> 

The .csv files with our results are provided as reference in `TTest_maze_results.csv`, `TTest_linear_results.csv`, and `TTest_open_results.csv`


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


