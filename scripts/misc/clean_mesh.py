import click
import pymeshlab

if __name__ == "__main__":

    @click.command(no_args_is_help=True)
    @click.option("--mesh", type=str, help="Mesh to clean up")
    @click.option("--smoothen", is_flag=True, default=False, help="Smoothen mesh after cleanup")
    def main(mesh: str, smoothen: bool):
        print("Loading mesh")
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(mesh)

        original_vertices = ms.current_mesh().vertex_number()
        original_faces = ms.current_mesh().face_number()

        # Clean up mesh using same steps as Section C.3 of ScanNet paper:
        # 1. Merge close vertices
        # 2. Remove connected components with fewer than 7500 triangles
        # 3. Run two rounds of quadric edge collapse decimation
        # 4. Optionally, smoothen the mesh

        print("Merging close vertices")
        ms.meshing_merge_close_vertices()

        print("Removing connected components with fewer than 7500 triangles")
        ms.meshing_remove_connected_component_by_face_number(mincomponentsize=7500)

        print("Running first decimation")
        ms.meshing_decimation_quadric_edge_collapse(targetperc=0.5, autoclean=True)

        print("Running second decimation")
        ms.meshing_decimation_quadric_edge_collapse(targetperc=0.5, autoclean=True)

        if smoothen:
            print("Smoothing mesh")
            ms.apply_coord_two_steps_smoothing()

        # Output new stats and save to file
        new_vertices = ms.current_mesh().vertex_number()
        new_faces = ms.current_mesh().face_number()

        print("Vertices:", original_vertices, "-->", new_vertices)
        print("Faces:", original_faces, "-->", new_faces)

        ms.save_current_mesh(mesh[:-4] + "_cleaned.obj")

    main()
