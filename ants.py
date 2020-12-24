# hamza, Guerabli, 20112229
# Michael, Plante, 20182677

import numpy as np
import random as rand
import copy


class Colony:
    class Ant:
        def __init__(self, colony):
            self.colony = colony
            self.pos = rand.randrange(self.colony.n)

            self.mem = np.zeros(self.colony.n)
            self.mem[self.pos] = 1

            self.path = [self.pos]
            self.cost = 0

        def reset(self, colony):
            self.__init__(colony)

        def __str__(self):
            return str(self.path) + ', cost : ' + str(self.cost)

        def __lt__(self, other):
            return self.cost < other.cost

        # Returns city to be travelled to from current position
        def policy(self):

            if rand.random() < self.colony.q_0:
                next_node = None
                higher_val = 0

                # Deterministic decision

                for i in range(self.colony.n):

                    if not self.mem[i]:
                        s = self.colony.tau[self.pos][i] * self.colony.eta(self.pos, i) ** self.colony.beta

                        if s > higher_val:
                            higher_val = s
                            next_node = i

                return next_node
            else:

                # Stochastic decision
                next_node = None
                higher_val = 0

                for i in range(self.colony.n):

                    if not self.mem[i]:
                        s = 0
                        for j in range(self.colony.n):

                            if self.mem[j] and j != self.pos:

                                s += self.colony.tau[self.pos][j] * self.colony.eta(self.pos, j) ** self.colony.beta
                        if s == 0 :
                            p = 0
                        else:
                            p = (self.colony.tau[self.pos][i] * self.colony.eta(self.pos, i) ** self.colony.beta) / s

                        if (p >= higher_val):
                            higher_val = p
                            next_node = i
                return next_node
                # Updates the local pheromones and position of ant

        # while keeping track of total cost and path
        def move(self):
            destination = self.policy()

            # local updating
            local_update = (1 - self.colony.alpha) * self.colony.tau[self.pos][
                destination] + self.colony.alpha * self.colony.tau_0
            self.colony.tau[self.pos][destination] = local_update
            self.colony.tau[destination][self.pos] = local_update
            # Change position
            self.mem[destination] = 1

            self.path.append(destination)

            self.cost += self.colony.adjMat[self.pos, destination]
            self.pos = destination

        # Updates the pheromone levels of ALL edges that form
        # the minimum cost loop at each iteration
        def globalUpdate(self):
            for i in range(len(self.path) - 1):
                global_update = (1 - self.colony.alpha) * self.colony.tau[i][i + 1] + self.colony.alpha * (
                            1 / self.cost)
                self.colony.tau[i][i + 1] = global_update
                self.colony.tau[i + 1][i] = global_update

            print(self)

    def __init__(self, adjMat, m=10, beta=2, alpha=0.1, q_0=0.9):
        # Parameters:
        # m => Number of ants
        # beta => Importance of heuristic function vs pheromone trail
        # alpha => Updating propensity
        # q_0 => Probability of making a non-stochastic decision
        # tau_0 => Initial pheromone level

        self.adjMat = adjMat
        self.n = len(adjMat)

        self.tau_0 = 1 / (self.n * self.nearestNearbourHeuristic())
        self.tau = [[self.tau_0 for _ in range(self.n)] for _ in range(self.n)]
        self.ants = [self.Ant(self) for _ in range(m)]

        self.beta = beta
        self.alpha = 0.1
        self.q_0 = q_0

    def __str__(self):
        string = ''
        for ant in self.ants:
            string += str(ant) + '\n'
        return string

    # Returns the cost of the solution produced by
    # the nearest neighbour heuristix
    def nearestNearbourHeuristic(self):
        costs = np.zeros(self.n)

        for i in range(self.n):
            x = copy.copy(self.adjMat[i])
            x.sort()
            costs[i] = x[1]

        return min(costs)

    # Heuristic function
    # Returns inverse of smallest distance between r and u
    def eta(self, r, u):
        return 1 / self.adjMat[r][u]

    def optimize(self, num_iter):
        for _ in range(num_iter):
            for _ in range(self.n - 1):
                for ant in self.ants:
                    ant.move()

            min(self.ants).globalUpdate()

            for ant in self.ants:
                ant.reset(self)


if __name__ == "__main__":
    rand.seed(420)

    # file = open('d198')
    file = open('dantzig.csv')

    adjMat = np.loadtxt(file, delimiter=",")

    ant_colony = Colony(adjMat)

    ant_colony.optimize(1000)
