# PyBodhi

The PyBodhi behaviour tree implementation operates by observing the state of a
system and expressing decisions base on it.
PyBodhi is designed in such a way that behaviour trees do no interact with a
system directly, but rather serve as an oracle to the system itself.
Instead of executing actions directly, PyBodhi modularises its behaviour tree
implementation by deciding which actions to execute.

# Installation

Simply clone this repository:
```bash
git clone https://github.com/cisprague/pybodhi.git
```
Enter the repository and install it into your system:
```bash
cd pybodhi
sudo pip3 install .
```
