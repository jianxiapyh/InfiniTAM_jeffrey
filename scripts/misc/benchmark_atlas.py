import open3d as o3d # pip install open3d --use-feature=2020-resolver
import numpy as np
import click


def nn_correspondance(verts1, verts2):
    distances = []
    if len(verts1) == 0 or len(verts2) == 0:
        return  distances

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(verts1)
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    for vert in verts2:
        _, _, dist = kdtree.search_knn_vector_3d(vert, 1)
        distances.append(np.sqrt(dist[0]))

    return np.array(distances)

if __name__ == "__main__":

    @click.command(no_args_is_help=True)
    @click.option("--file_pred", type=str, help="Mesh")
    @click.option("--file_trgt", type=str, help="Ground Truth")
    @click.option("--num_pcd", type=int, default=0, help="number of point cloud")
    def main(file_pred, file_trgt, num_pcd, threshold=.05):

        print("Reading mesh from files")
        mesh_pred = o3d.io.read_triangle_mesh(file_pred)
        mesh_trgt = o3d.io.read_triangle_mesh(file_trgt)

        if num_pcd != 0:
            print(f'Using {num_pcd} number of point clouds')
            pcd_pred = mesh_pred.sample_points_uniformly(number_of_points=num_pcd)
            pcd_trgt = mesh_trgt.sample_points_uniformly(number_of_points=num_pcd)
            verts_pred = np.asarray(pcd_pred.points)
            verts_trgt = np.asarray(pcd_trgt.points)
        else:
            print(f'Using vertices as point clouds {len(mesh_pred.vertices)}')
            verts_pred = np.asarray(mesh_pred.vertices)
            verts_trgt = np.asarray(mesh_trgt.vertices)

        dist1 = nn_correspondance(verts_pred, verts_trgt) # for each v in groundtruth, find nearest vertex in predicted
        print(f'comp: {np.mean(dist1)}')

        dist2 = nn_correspondance(verts_trgt, verts_pred) # for each v in predicted, find nearest vertex in groundtruth
        print(f'acc: {np.mean(dist2)}')

        precision = np.mean((dist2<threshold).astype('float'))
        print(f'prec: {precision}')

        recal = np.mean((dist1<threshold).astype('float'))
        print(f'recal: {recal}')

        fscore = 2 * precision * recal / (precision + recal)
        print(f'fscore: {fscore}')

    main()


