# A comparison of Depth First Search, Uniform Cost Search and Astar algorithm  

*Author: Kexuan Zhai*

<!-- ABOUT THE PROJECT -->
## About The Project

<div align=center><img width="500" height="375" src="https://github.com/ZhaiKexuan/Motion-Planning/blob/master/Comparison-of-DFS-UCS-and-Astar/images/image1.png"/></div>
In this project, I implemented a number of search algorithms to guide an agent through a grid-based world. For simplicity, we'll assume that the state space is 2-dimensional, consisting of the grid location of the agent.

There following method is covered:
* Depth-first search algorithm
<div align=center><img width="500" height="375" src="https://github.com/ZhaiKexuan/Motion-Planning/blob/master/Comparison-of-DFS-UCS-and-Astar/images/image1.png"/></div>

* Uniform_cost_search algorithm
<div align=center><img width="500" height="375" src="https://github.com/ZhaiKexuan/Motion-Planning/blob/master/Comparison-of-DFS-UCS-and-Astar/images/image2.png"/></div>

* A* graph search algorithm
<div align=center><img width="500" height="375" src="https://github.com/ZhaiKexuan/Motion-Planning/blob/master/Comparison-of-DFS-UCS-and-Astar/images/image3.png"/></div>

### Built With
* [Python 3.6](https://www.python.org/downloads/release/python-360/)
* [Matplotlib](https://matplotlib.org/)

<!-- GETTING STARTED -->
## Installation
1. Clone the repo
```sh
git clone https://github.com/ZhaiKexuan/Motion-Planning/tree/master/Comparison-of-DFS-UCS-and-Astar
```
2. Install matplotlib
```sh
pip install matplotlib
```
3. display the grid world and generate different maps through the GUI
```sh
python search.py
```