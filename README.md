To generate random layouts:
python createRandom.py -width 15 -height 15 -n 1 -e

arguments:
width - width of the maze
heigth - height of the maze
n - number of mazes to generate
e - whether to make the maze open/empty. omit the flag to generate walled mazes

examples:
open layouts - python createRandom.py -width 15 -height 15 -n 50 -e
maze layouts - python createRandom.py -width 15 -height 15 -n 50
lienar layouts - python createRandom.py -width 2 -height 15 -n 50

To run analysis:
python pacman.py -p SearchAgent --randomtest 1 --frameTime 0 --quietTextGraphics

This will run every algorithm for every layout in layouts/random. Different sets of randomizes layouts are provided in layouts/randomOpen, layouts/randomMaze, and layouts/randomLinear. These sets can be copied into layouts/random for testing. Additionally, a one way T-test is conducted between every algorithm pair on the nodes expanded. The results generate in the TTest_results.csv file in the root folder. If the random folder is empty an error will result.