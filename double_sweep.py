import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp

# Import directly from SimEngine (not main)
from SimEngine import Engine
from setups import Panda

def double_sweep_parallel():
    # Initialize engine
    engine = Engine(trajectory="./trajectory/23_michigan_endurance_ft.csv", is_closed=True)

    # Parameter ranges
    Cd_range = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
    weight_range = np.linspace(-50, 20, 8)

    # Build parameter grid
    tasks = []
    for w in weight_range:
        for Cd in Cd_range:
            overrides = {"Cd": Cd, "total_weight": 595 + w}
            vehicle_params = Panda(overrides=overrides)
            payload = {"PARAMS": vehicle_params, "COUNT": 0}
            tasks.append(payload)

    # Parallel execution
    with mp.Pool(processes=mp.cpu_count() - 1) as pool:
        laptimes = pool.map(engine.compute_task_authoritative, tasks)

    # Convert to 2D grid
    laptimes = np.array(laptimes).reshape(len(weight_range), len(Cd_range))
    
    # Save results to CSV
    import csv
    with open("laptime_sweep_Cd_AddedWeight.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Cd", "Added Weight", "Laptime"])
        for i, w in enumerate(weight_range):
            for j, Cd in enumerate(Cd_range):
                writer.writerow([Cd, w, laptimes[i, j]])

    # 3D surface plot
    Cd_grid, W_grid = np.meshgrid(Cd_range, weight_range)
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(Cd_grid, W_grid, laptimes, cmap="viridis", edgecolor="k", linewidth=0.3)
    ax.set_xlabel("Cd", labelpad=10)
    ax.set_ylabel("Added Weight (lbs)", labelpad=10)
    ax.set_zlabel("Laptime (s)", labelpad=10)
    ax.set_title("Laptime vs Cd and Added Weight (Endurance Mode)")
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label="Laptime (s)")
    plt.show()

if __name__ == "__main__":
    double_sweep_parallel()
