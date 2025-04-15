import numpy as np
import random
from scipy.spatial import distance
import pandas as pd
import time
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def load_data(csv_file_path):
    df = pd.read_csv(csv_file_path)
    return df


def identify_difficulty_areas(df):
    difficulty_areas = []
    cardboard_box_y_threshold = 0.35
    wooden_case_x_range = (0.3, 0.55)
    wooden_case_y_range = (-0.8, -0.5)

    for _, row in df.iterrows():
        if row["Y"] > cardboard_box_y_threshold or (
                wooden_case_x_range[0] <= row["X"] <= wooden_case_x_range[1] and
                wooden_case_y_range[0] <= row["Y"] <= wooden_case_y_range[1]
        ):
            difficulty_areas.append(row["Object"])
    return difficulty_areas


def fitness_function(sequence, df, score_dict, difficulty_areas, v, beta, T_max, difficulty_penalty=0.3):
    total_score = 0
    total_time = 0
    robot_position = [0, 0, 0]

    for obj in sequence:
        obj_data = df[df["Object"] == obj].iloc[0]
        obj_type = obj_data["Type"]
        obj_position = [obj_data["X"], obj_data["Y"], obj_data["Z"]]

        score = score_dict.get(obj_type, 0)
        if obj in difficulty_areas:
            score *= difficulty_penalty

        dist = distance.euclidean(robot_position, obj_position)
        move_time = dist / v + beta

        if total_time + move_time > T_max:
            break

        total_score += score
        total_time += move_time
        robot_position = obj_position

    return total_score


def truncate_sequence_by_time(sequence, df, score_dict, difficulty_areas, v, beta, T_max, difficulty_penalty=0.3):
    total_time = 0
    robot_position = [0, 0, 0]
    valid_sequence = []

    for obj in sequence:
        obj_data = df[df["Object"] == obj].iloc[0]
        obj_type = obj_data["Type"]
        obj_position = [obj_data["X"], obj_data["Y"], obj_data["Z"]]

        dist = distance.euclidean(robot_position, obj_position)
        move_time = dist / v + beta

        if total_time + move_time > T_max:
            break

        total_time += move_time
        robot_position = obj_position
        valid_sequence.append(obj)

    return valid_sequence


def generate_population(df, size):
    population = []
    objects_list = df["Object"].tolist()
    for _ in range(size):
        random.shuffle(objects_list)
        population.append(objects_list[:])
    return population


def selection(population, df, score_dict, difficulty_areas, v, beta, T_max):
    tournament_size = 5
    selected = random.sample(population, tournament_size)
    selected = sorted(selected, key=lambda x: fitness_function(x, df, score_dict, difficulty_areas, v, beta, T_max),
                      reverse=True)
    return selected[0]


def optimized_crossover(parent1, parent2):
    size = len(parent1)
    cut = random.randint(1, size - 1)
    child1 = parent1[:cut] + [gene for gene in parent2 if gene not in parent1[:cut]]
    child2 = parent2[:cut] + [gene for gene in parent1 if gene not in parent2[:cut]]
    return child1, child2


def mutate(sequence):
    idx1, idx2 = random.sample(range(len(sequence)), 2)
    sequence[idx1], sequence[idx2] = sequence[idx2], sequence[idx1]


def run_genetic_algorithm(csv_file_path, num_generations, population_size, v, beta, T_max):
    df = load_data(csv_file_path)
    difficulty_areas = identify_difficulty_areas(df)

    score_dict = {
        "redBottle": 7, "yellowBottle": 6, "blueBottle": 5,
        "redCan": 5, "yellowCan": 4, "greenCan": 3,
        "wood_cube": 2
    }

    start_time = time.time()
    population = generate_population(df, population_size)

    for _ in range(num_generations):
        new_population = []
        for _ in range(population_size // 2):
            parent1 = selection(population, df, score_dict, difficulty_areas, v, beta, T_max)
            parent2 = selection(population, df, score_dict, difficulty_areas, v, beta, T_max)
            child1, child2 = optimized_crossover(parent1, parent2)

            if random.random() < 0.5:
                mutate(child1)
            if random.random() < 0.5:
                mutate(child2)

            new_population.extend([child1, child2])

        population = sorted(new_population,
                            key=lambda x: fitness_function(x, df, score_dict, difficulty_areas, v, beta, T_max),
                            reverse=True)[:population_size]

    best_sequence = population[0]
    end_time = time.time()
    execution_time = end_time - start_time

    return best_sequence, execution_time, df, score_dict, difficulty_areas


def save_results(best_sequence, file_name="best_grasp_order.csv"):
    pd.DataFrame(best_sequence, columns=["Grasp Order"]).to_csv(file_name, index=False)
    print(pd.read_csv(file_name))


def visualize_sequence(best_sequence, df):
    fig, ax = plt.subplots()
    bin_coords = {
        "Blue Bin": (-0.4652, -0.5901),
        "Green Bin": (-0.4654, 0.3673)
    }
    all_x, all_y = [], []
    current = (0, 0)

    for i, obj in enumerate(best_sequence):
        obj_data = df[df["Object"] == obj].iloc[0]
        x, y = obj_data["X"], obj_data["Y"]
        obj_type = obj_data["Type"]
        all_x.append(x)
        all_y.append(y)

        bin_target = "Blue Bin" if "Bottle" in obj_type else "Green Bin"
        bx, by = bin_coords[bin_target]
        all_x.append(bx)
        all_y.append(by)

        ax.annotate('', xy=(x, y), xytext=current, arrowprops=dict(arrowstyle="->", color='blue'))
        ax.annotate(str(i + 1), (x, y), textcoords="offset points", xytext=(0, 5), ha='center', fontsize=8,
                    color='blue')
        ax.annotate('', xy=(bx, by), xytext=(x, y), arrowprops=dict(arrowstyle="->", color='gray'))
        current = (bx, by)

    for name, (x, y) in bin_coords.items():
        ax.plot(x, y, 'ro')
        ax.text(x, y + 0.03, name, ha='center', fontsize=9, color='red')

    margin = 0.1
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Grasping Sequence Visualization")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    csv_file_path = "Arm_Object_positions.csv"
    v = 0.5
    beta = 10
    T_max = 180
    generations = 24
    population_size = 20

    best_sequence, exec_time, df, score_dict, difficulty_areas = run_genetic_algorithm(
        csv_file_path, generations, population_size, v, beta, T_max)

    valid_sequence = truncate_sequence_by_time(best_sequence, df, score_dict, difficulty_areas, v, beta, T_max)
    final_fitness = fitness_function(valid_sequence, df, score_dict, difficulty_areas, v, beta, T_max)

    save_results(valid_sequence)
    print("Final Valid Grasping Sequence:", valid_sequence)
    print("Final Fitness Score (cutoff applied):", final_fitness)
    print("Execution Time:", exec_time, "seconds")

    visualize_sequence(valid_sequence, df)
