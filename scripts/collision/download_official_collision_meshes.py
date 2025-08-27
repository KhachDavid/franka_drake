#!/usr/bin/env python3
"""
Download official Franka collision meshes from franka_description repository
and convert them from STL to OBJ format for Drake compatibility.
"""

import os
import sys
import urllib.request
import subprocess
from pathlib import Path

def download_file(url, local_path):
    """Download a file from URL to local path."""
    print(f"Downloading {url} -> {local_path}")
    try:
        urllib.request.urlretrieve(url, local_path)
        return True
    except Exception as e:
        print(f"Error downloading {url}: {e}")
        return False

def convert_stl_to_obj(stl_path, obj_path):
    """Convert STL to OBJ using meshlab or blender if available."""
    stl_path = Path(stl_path)
    obj_path = Path(obj_path)
    
    # Try meshlab first (meshlabserver)
    try:
        result = subprocess.run([
            'meshlabserver', '-i', str(stl_path), '-o', str(obj_path)
        ], capture_output=True, text=True, check=True)
        print(f"Converted {stl_path} -> {obj_path} using meshlab")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Try blender
    try:
        result = subprocess.run([
            'blender', '--background', '--python-expr',
            f"import bpy; bpy.ops.import_mesh.stl(filepath='{stl_path}'); bpy.ops.export_scene.obj(filepath='{obj_path}')"
        ], capture_output=True, text=True, check=True)
        print(f"Converted {stl_path} -> {obj_path} using blender")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Try simple Python conversion using numpy-stl and trimesh
    try:
        import numpy as np
        from stl import mesh
        import trimesh
        
        # Load STL
        stl_mesh = mesh.Mesh.from_file(str(stl_path))
        vertices = stl_mesh.vectors.reshape(-1, 3)
        
        # Remove duplicate vertices
        unique_vertices, indices = np.unique(vertices, axis=0, return_inverse=True)
        faces = indices.reshape(-1, 3)
        
        # Create trimesh object and export
        tm = trimesh.Trimesh(vertices=unique_vertices, faces=faces)
        tm.export(str(obj_path))
        print(f"Converted {stl_path} -> {obj_path} using python libraries")
        return True
    except ImportError:
        print("numpy-stl and trimesh not available for conversion")
    except Exception as e:
        print(f"Python conversion failed: {e}")
    
    print(f"WARNING: Could not convert {stl_path} to OBJ format")
    print("Please install one of: meshlab, blender, or 'pip install numpy-stl trimesh'")
    return False

def main():
    # Get script directory and set up paths
    script_dir = Path(__file__).parent
    models_dir = script_dir.parent / "models"
    collision_dir = models_dir / "meshes" / "robot_arms" / "fer" / "collision"
    hand_collision_dir = models_dir / "meshes" / "robot_ee" / "franka_hand_white" / "collision"
    
    # Create directories
    collision_dir.mkdir(parents=True, exist_ok=True)
    hand_collision_dir.mkdir(parents=True, exist_ok=True)
    
    # Base URLs for official Franka meshes
    base_url = "https://raw.githubusercontent.com/frankarobotics/franka_description/main"
    fr3_collision_url = f"{base_url}/meshes/robot_arms/fr3/collision"
    hand_collision_url = f"{base_url}/meshes/robot_ee/franka_hand_white/collision"
    
    # FR3 collision meshes to download
    fr3_meshes = [
        "link0.stl", "link1.stl", "link2.stl", "link3.stl", 
        "link4.stl", "link5.stl", "link6.stl", "link7.stl"
    ]
    
    # Hand collision mesh
    hand_meshes = ["hand.stl"]
    
    success_count = 0
    total_count = 0
    
    # Download FR3 collision meshes
    print("Downloading FR3 collision meshes...")
    for mesh_file in fr3_meshes:
        url = f"{fr3_collision_url}/{mesh_file}"
        stl_path = collision_dir / mesh_file
        obj_path = collision_dir / mesh_file.replace(".stl", ".obj")
        
        total_count += 1
        if download_file(url, stl_path):
            if convert_stl_to_obj(stl_path, obj_path):
                # Remove STL file after successful conversion
                stl_path.unlink()
                success_count += 1
            else:
                print(f"Keeping STL file: {stl_path}")
    
    # Download hand collision mesh
    print("\nDownloading hand collision mesh...")
    for mesh_file in hand_meshes:
        url = f"{hand_collision_url}/{mesh_file}"
        stl_path = hand_collision_dir / mesh_file
        obj_path = hand_collision_dir / mesh_file.replace(".stl", ".obj")
        
        total_count += 1
        if download_file(url, stl_path):
            if convert_stl_to_obj(stl_path, obj_path):
                # Remove STL file after successful conversion
                stl_path.unlink()
                success_count += 1
            else:
                print(f"Keeping STL file: {stl_path}")
    
    print(f"\nCompleted: {success_count}/{total_count} meshes successfully downloaded and converted")
    
    if success_count == total_count:
        print("✓ All official collision meshes ready for use!")
    else:
        print("⚠ Some meshes may need manual conversion from STL to OBJ")
        print("Install meshlab, blender, or run: pip install numpy-stl trimesh")

if __name__ == "__main__":
    main()
