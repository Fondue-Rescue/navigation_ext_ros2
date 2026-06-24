#!/usr/bin/env python3
import sys
import os
import math
import time

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 regenerate_correspondences.py <search_radius>")
        print("Example: python3 regenerate_correspondences.py 0.30")
        sys.exit(1)
        
    try:
        search_radius = float(sys.argv[1])
    except ValueError:
        print("Error: search_radius must be a float number.")
        sys.exit(1)

    # Paths are relative to the script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    paths_ply_path = os.path.join(script_dir, 'paths.ply')
    output_path = os.path.join(script_dir, 'correspondences.txt')

    if not os.path.exists(paths_ply_path):
        print(f"Error: {paths_ply_path} not found.")
        sys.exit(1)

    offsetX = 3.2
    offsetY = 4.5
    voxelSize = 0.02
    voxelNumX = 161
    voxelNumY = 451

    start_time = time.time()
    print(f"Reading {paths_ply_path}...")
    path_points = []
    with open(paths_ply_path, 'r') as f:
        # Skip PLY header
        line = f.readline()
        while line and not line.startswith('end_header'):
            line = f.readline()
        
        # Read vertices
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                x = float(parts[0])
                y = float(parts[1])
                path_id = int(parts[3])
                path_points.append((x, y, path_id))
    
    print(f"Loaded {len(path_points)} path points in {time.time() - start_time:.2f} seconds.")

    bin_start = time.time()
    # We use cell_size equal to the search radius to keep the neighborhood search
    # window to exactly 3x3 cells (9 cells total), which minimizes dictionary lookups.
    cell_size = search_radius
    grid = {}
    
    def get_cell(x, y):
        return (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))

    print(f"Binning path points into grid of cell size {cell_size:.3f}m...")
    for x, y, path_id in path_points:
        cell = get_cell(x, y)
        if cell not in grid:
            grid[cell] = []
        grid[cell].append((x, y, path_id))
    print(f"Binned in {time.time() - bin_start:.2f} seconds.")

    search_start = time.time()
    print(f"Generating 72,611 voxel points and checking collisions (radius={search_radius:.3f}m)...")
    idx_voxel = 0
    r_sq = search_radius * search_radius
    
    # Pre-allocate buffer for output
    with open(output_path, 'w') as out_f:
        for indX in range(voxelNumX):
            x = offsetX - voxelSize * indX
            # scaleY formulas from path_generator.m
            scaleY = x / offsetX + search_radius / offsetY * (offsetX - x) / offsetX
            for indY in range(voxelNumY):
                y = scaleY * (offsetY - voxelSize * indY)
                
                cx, cy = get_cell(x, y)
                nearby_paths = set()
                
                # Check 3x3 window of cells
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        cell = (cx + dx, cy + dy)
                        if cell in grid:
                            for px, py, path_id in grid[cell]:
                                dist_sq = (px - x)**2 + (py - y)**2
                                if dist_sq <= r_sq:
                                    nearby_paths.add(path_id)
                
                sorted_paths = sorted(list(nearby_paths))
                paths_str = " ".join(str(p) for p in sorted_paths)
                if paths_str:
                    out_f.write(f"{idx_voxel} {paths_str} -1\n")
                else:
                    out_f.write(f"{idx_voxel} -1\n")
                idx_voxel += 1
                
                if idx_voxel % 15000 == 0:
                    print(f"Processed {idx_voxel}/72611 voxels...")
                
    print(f"Collision checks finished in {time.time() - search_start:.2f} seconds.")
    print(f"Successfully saved regenerated lookup table to: {output_path}")

if __name__ == '__main__':
    main()
