# chat GPT version

import numpy as np


def hungarian_algorithm2(cost_matrix):
    num_tasks = min(cost_matrix.shape[0], 4)  # Maximum 4 tasks
    print(num_tasks)
    num_robots = cost_matrix.shape[1]
    print(num_robots)
    # Pad the cost matrix with large values if the number of tasks is less than 4
    if num_tasks < 4:
        padded_cost_matrix = np.pad(cost_matrix, ((0, 4 - num_tasks), (0, 0)), constant_values=np.inf)
    else:
        padded_cost_matrix = cost_matrix.copy()
    # Step 1: Subtract the minimum value in each row from all elements in the row
    subtracted_rows = padded_cost_matrix - np.min(padded_cost_matrix, axis=1)[:, np.newaxis]
    # Step 2: Subtract the minimum value in each column from all elements in the column
    subtracted_matrix = subtracted_rows - np.min(subtracted_rows, axis=0)
    # Step 3: Assign tasks to robots using the modified Hungarian algorithm
    assignment = np.zeros(num_robots, dtype=int) - 1  # Initialize assignment array
    for task in range(num_tasks):
        visited_rows = np.zeros(num_tasks, dtype=bool)
        visited_cols = np.zeros(num_robots, dtype=bool)
        if augment_path(task, visited_rows, visited_cols, assignment, subtracted_matrix):
            break
    # Return the assignment positions
    assignment_positions = [(task, robot) for robot, task in enumerate(assignment) if task != -1]
    return assignment_positions

def augment_path(task, visited_rows, visited_cols, assignment, subtracted_matrix):
    num_robots = subtracted_matrix.shape[1]
    for robot in range(num_robots):
        if not visited_cols[robot] and subtracted_matrix[task][robot] == 0:
            visited_cols[robot] = True
            if assignment[robot] == -1 or augment_path(assignment[robot], visited_rows, visited_cols, assignment, subtracted_matrix):
                assignment[robot] = task
                return True
    return False












