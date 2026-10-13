"""Candidate solution for the assignment "Map, Data Association & Path Planning".

Run it with::

    python3 solution.py
The script executes the full pipeline step by step and stops at the first
step you have not implemented yet, with a reminder of what is missing.
Implement the three functions below (``associate``, ``correct_map``,
``find_path``), keeping their names and signatures unchanged.
"""

import math
from typing import List, Tuple
import random
from map_generation import Map, Point, generate_map, generate_observations
from visualization import (
    visualize_association_n_correction,
    visualize_map_n_observations,
    visualize_path,
)

# ---------------------------------------------------------------------------
# Parameters of the run.
#
# Feel free to experiment with them, but note that your solution will also be
# evaluated with different values (especially with different random seeds).
# ---------------------------------------------------------------------------
NUM_POINTS = 20
X_RANGE = 20.0
Y_RANGE = 20.0
MAP_SEED = 42

NOISE_STD = 0.25
OBSERVATION_RATE = 0.8
OBSERVATIONS_SEED = 7


# ---------------------------------------------------------------------------
# Provided helper
# ---------------------------------------------------------------------------

def path_length(points: List[Point], path: List[int]) -> float:
    """Total Euclidean length of ``path`` over ``points``.

    Useful to measure how short your path is, e.g. when going for Bonus 1.
    The length is the sum of the Euclidean distances between the consecutive
    nodes of the path.
    """
    return sum(math.dist(points[a], points[b]) for a, b in zip(path, path[1:]))
# ---------------------------------------------------------------------------
# Step 3 -- associations        (implemented)
# ---------------------------------------------------------------------------

def associate(map_points: Map, observations: List[Point]) -> List[Tuple[int, int]]:
    """To solve the task i created an algorithm that first create a new list that contains
    Tuples of points and their index. """

    remaining_observations = list(enumerate(observations))
    remaining_map = list(enumerate(map_points))
    # the empty list that will contain associations
    associations = []

    """to associate every observation index to a map index, i compute every possible distance between
       each point of remaining observations and remaining map. After that I find the min_distance,I append these points' indexes
        to associations and to avoid repetitions before the next iteration i delete these points and their indexes from remainingmap
        and remaing observation. Since it s possible that there are more map_points than observations when I have associated each observation
        I store the remaing map indexes in asociations.

        """
    for i in range(len(observations)):

        min_distance = 100000 # to inizialise min_distance i chose a very large number, so it s gonna be surely changed during the first iteration
        best_map_index = None
        best_observation_index = None


        for map_index, map_point in remaining_map:
            for observation_index, observation_point in remaining_observations:

                distance = (
                    (map_point[0] - observation_point[0])**2
                    + (map_point[1] - observation_point[1])**2
                )**0.5


                if distance < min_distance:
                    min_distance = distance
                    best_map_index = map_index
                    best_observation_index = observation_index
                    best_map_point = map_point
                    best_observation_point = observation_point


        associations.append((best_map_index, best_observation_index))

        map_tuple = (best_map_index, best_map_point)
        remaining_map.remove(map_tuple)
        observations_tuple = (best_observation_index, best_observation_point)
        remaining_observations.remove(observations_tuple)


    for map_index, point in remaining_map:
        associations.append((map_index, map_index))

    return associations

# ---------------------------------------------------------------------------
# Step 4 -- Map correction (implemented)
# ---------------------------------------------------------------------------


def correct_map(
    map_points: Map,
    observations: List[Point],
    associations: List[Tuple[int, int]],
) -> Map:
    """As requested by the task I create the car's updated internal map. I set the new position of each landmark
    to the midpoint of its previous map position and its observed position."""
    corrected_map = list(map_points)

    for i, j in associations:
      if i!=j: #Thanks to this map's points that were not observed don't change their position
        x_map, y_map = map_points[i]
        x_obs, y_obs = observations[j]

        corrected_map[i] = (
            (x_map + x_obs) / 2,
            (y_map + y_obs) / 2,
        )
      else:
        corrected_map[i] = map_points[i]

    return corrected_map


# ---------------------------------------------------------------------------
# Step 5 -- Path finding (implemented)
# ---------------------------------------------------------------------------

def find_path(corrected_map: Map) -> List[int]:
    """I start the algorithm by choosing a random element as the starting point, first i store it in the path list and delete it
    from the unvisited list, then  i compure every distance between this point and the others. I choose the min distance and store the 
    new point i found in path, delete it from the unvisited list and repeat the iteration...  """
    n = len(corrected_map)

    start = random.randrange(n)

    path = [start]
    unvisited = list(range(n))
    unvisited.remove(start)

    while len(unvisited) > 0:
        current = path[-1]

        min_distance = 100000 # to inizialise min_distance i chose a very large number, so it s gonna be surely changed during the first iteration
        next_point = None


        for i in unvisited:
            distance = math.dist(corrected_map[current], corrected_map[i] )

            if distance < min_distance:
                min_distance = distance
                next_point = i


        path.append(next_point)

        unvisited.remove(next_point)

    return path

# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def main() -> None:
    # Step 1 -- Generate the map (provided) ---------------------------------
    map_points = generate_map(
        num_points=NUM_POINTS,
        x_range=X_RANGE,
        y_range=Y_RANGE,
        seed=MAP_SEED,
    )

    # Step 2 -- Generate the observations (provided) ------------------------
    observations = generate_observations(
        map_points,
        noise_std=NOISE_STD,
        observation_rate=OBSERVATION_RATE,
        seed=OBSERVATIONS_SEED,
    )
    visualize_map_n_observations(map_points, observations, X_RANGE, Y_RANGE)

    # Step 3 -- Data association (yours) -------------------------------------
    try:
        associations = associate(map_points, observations)
    except NotImplementedError:
        print("Step 3 is not implemented yet: fill in associate() in solution.py.")
        return

    # Step 4 -- Map correction (yours) ---------------------------------------
    try:
        corrected_map = correct_map(map_points, observations, associations)
    except NotImplementedError:
        print("Step 4 is not implemented yet: fill in correct_map() in solution.py.")
        return
    visualize_association_n_correction(
        map_points,
        observations,
        corrected_map,
        associations,
        X_RANGE,
        Y_RANGE,
    )

    # Step 5 -- Path finding (yours) -----------------------------------------
    try:
        path = find_path(corrected_map)
    except NotImplementedError:
        print("Step 5 is not implemented yet: fill in find_path() in solution.py.")
        return
    visualize_path(corrected_map, path)

    print(
        f"Pipeline complete: {len(associations)} associations over "
        f"{len(observations)} observations, path over {len(corrected_map)} "
        f"nodes with total length {path_length(corrected_map, path):.2f}."
    )


if __name__ == "__main__":
    main()
