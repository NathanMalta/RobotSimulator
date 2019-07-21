"""
Single-pole balancing experiment using a feed-forward neural network.
"""

from __future__ import print_function

import os
import pickle
import statistics

import neat
import time

from Simulator import Simulator
from SimulationRenderer import SimulationRenderer
import visualize

runs_per_net = 3
generations = 150


# Use the NN network phenotype and the discrete actuator force function.
def eval_genome(genome, config):
	net = neat.nn.FeedForwardNetwork.create(genome, config)

	fitnesses = []

	for runs in range(runs_per_net):
		sim = Simulator(testNum=runs)

		# Run the given simulation for up to num_steps time steps.
		while sim.shouldContinue():
			inputs = sim.getInputActivations()
			action = net.activate(inputs)

			# Apply action to the simulated cart-pole
			sim.update(action[0], action[1])

		fitnesses.append(sim.getFitness())

	# The genome's fitness is its worst performance across all runs.
	return statistics.mean(fitnesses)


def eval_genomes(genomes, config):
	for genome_id, genome in genomes:
		genome.fitness = eval_genome(genome, config)


def run():
	# Load the config file, which is assumed to live in
	# the same directory as this script.
	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')
	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						 neat.DefaultSpeciesSet, neat.DefaultStagnation,
						 config_path)

	pop = neat.Population(config)
	stats = neat.StatisticsReporter()
	pop.add_reporter(stats)
	pop.add_reporter(neat.StdOutReporter(True))

	pe = neat.ParallelEvaluator(2, eval_genome)
	winner = pop.run(pe.evaluate, generations)
	print(winner)

	winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
	# Save the winner.
	with open('winner-feedforward', 'wb') as f:
		pickle.dump(winner_net, f)

	for _ in range(10):
		simWindow = SimulationRenderer(False, winner_net)

	visualize.plot_stats(stats, ylog=True, view=True, filename="feedforward-fitness.svg")
	visualize.plot_species(stats, view=True, filename="feedforward-speciation.svg")

	node_names = {-1: 'headingDiff', -2: 'headingReq', -3: 'targetRoboCentric X', -4: 'targetRoboCentric Y', -5: 'distance', 0: 'rightMotor', 1: 'leftMotor'}
	visualize.draw_net(config, winner, True, node_names=node_names)

	visualize.draw_net(config, winner, view=True, node_names=node_names,
					   filename="winner-feedforward.gv")
	visualize.draw_net(config, winner, view=True, node_names=node_names,
					   filename="winner-feedforward-enabled.gv", show_disabled=False)
	visualize.draw_net(config, winner, view=True, node_names=node_names,
					   filename="winner-feedforward-enabled-pruned.gv", show_disabled=False, prune_unused=True)


if __name__ == '__main__':
	run()