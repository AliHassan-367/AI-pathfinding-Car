import pygame
import random
import heapq

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1080, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("AI Car Pathfinding Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (200, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)
GRAY = (180, 180, 180)

# Car settings
car_size = (40, 20)

# Grid settings
grid_size =  50# Increased to make obstacles larger

# Function to get user-defined start and end positions
def get_user_positions():
    screen.fill(GRAY)
    pygame.display.flip()

    print("Click to select the START position for the car.")
    start_selected = False
    start_pos = None
    while not start_selected:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                start_pos = (event.pos[0] // grid_size * grid_size, event.pos[1] // grid_size * grid_size)
                start_selected = True

    print("Click to select the END position (destination).")
    end_selected = False
    end_pos = None
    while not end_selected:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                end_pos = (event.pos[0] // grid_size * grid_size, event.pos[1] // grid_size * grid_size)
                end_selected = True

    return start_pos, end_pos

# Function to generate large obstacles
def generate_obstacles(num):
    obstacles = set()
    for _ in range(num):
        x = random.randint(1, WIDTH // grid_size - 2) * grid_size
        y = random.randint(1, HEIGHT // grid_size - 2) * grid_size
        obstacles.add((x, y))
    return obstacles

# A* Pathfinding Algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_pathfinding(start, goal, obstacles):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            break

        for dx, dy in [(0, -grid_size), (0, grid_size), (-grid_size, 0), (grid_size, 0)]:
            next_node = (current[0] + dx, current[1] + dy)
            if (0 <= next_node[0] < WIDTH and 0 <= next_node[1] < HEIGHT and
                next_node not in obstacles):
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    heapq.heappush(open_list, (priority, next_node))
                    came_from[next_node] = current

    path = []
    current = goal
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# Get user-defined start and end positions
start_pos, goal_pos = get_user_positions()

# Generate obstacles
num_obstacles = 100  # Reduced slightly since obstacles are bigger
obstacles = generate_obstacles(num_obstacles)

# Ensure obstacles do not overlap with start or goal
obstacles.discard(start_pos)
obstacles.discard(goal_pos)

# Find path using A* Algorithm
path = a_star_pathfinding(start_pos, goal_pos, obstacles)

# Game loop
running = True
car_pos = list(start_pos)

while running:
    screen.fill(GRAY)  # Background is now visible

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draw obstacles (bigger size)
    for obs in obstacles:
        pygame.draw.rect(screen, BLACK, (obs[0], obs[1], grid_size, grid_size))

    # Draw path
    for node in path:
        pygame.draw.rect(screen, BLUE, (node[0] + 5, node[1] + 5, grid_size - 10, grid_size - 10))

    # Move car along path
    if path:
        car_pos = list(path.pop(0))

    # Draw goal
    pygame.draw.circle(screen, GREEN, (goal_pos[0] + grid_size // 2, goal_pos[1] + grid_size // 2), grid_size // 2)

    # Draw car
    pygame.draw.rect(screen, RED, (car_pos[0], car_pos[1], car_size[0], car_size[1]))  # Car body
    pygame.draw.circle(screen, BLACK, (car_pos[0] + 5, car_pos[1] + car_size[1] + 5), 5)  # Wheels
    pygame.draw.circle(screen, BLACK, (car_pos[0] + car_size[0] - 5, car_pos[1] + car_size[1] + 5), 5)
    pygame.draw.circle(screen, BLACK, (car_pos[0] + 5, car_pos[1] - 5), 5)
    pygame.draw.circle(screen, BLACK, (car_pos[0] + car_size[0] - 5, car_pos[1] - 5), 5)

    # Update display
    pygame.display.flip()
    pygame.time.delay(200)

pygame.quit()
