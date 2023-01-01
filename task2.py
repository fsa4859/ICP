import copy
import cv2
import numpy as np
import open3d as o3d
import scipy
import sklearn
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
from sklearn.neighbors import NearestNeighbors

def Transformation(indices,source,target):
    '''
    returns the transformation between the source and target arrays
    based on the new correspondences.
    '''
    #source_corresp=source[indices,:]
    #print(f'the shape of source correspondences is{source_corresp.shape}')
    print(f'the shape of target correspondences is{target.shape}')
    mean_target=np.mean(target,axis=0)
    #mean_target=mean_target.reshape((3,1))
    print(f'shape of mean target is{mean_target.shape}')
    mean_source=np.mean(source,axis=0)
    #mean_source=mean_source.reshape((3,1))
    print(f'shape of mean source is{mean_source.shape}')
    # now subtract the mean from source and target
    centroid_source=source-mean_source
    centroid_target=target-mean_target
    print(f'the shape of centroid source is{centroid_source.shape}')
    print(f'the shape of centroid target is{centroid_target.shape}')
    # now form the H matrix
    H=np.dot(centroid_source.T,centroid_target)
    #print(f'the shape of H matrix is {H.shape} and is equal to {H}')
    # now form the R and t matrices
    u,s,vh=np.linalg.svd(H)
    #R,scale=scipy.linalg.orthogonal_procrustes(centroid_source,centroid_target)
    R=np.matmul(u,vh)
    print(f'The rotation matrix is {R} and of shape {R.shape}')
    # find translation vector
    t=mean_target.reshape((3,1))-np.matmul(R,mean_source.reshape((3,1)))
    print(f'The translation vector is {t} and of shape {t.shape}')
    T = np.identity(4)
    T[:3, :3] = R
    T[:3,3]=t.reshape((3,))
    print(T)
    return T,R,t,centroid_source,centroid_target,mean_target


def find_correspondences(source_pc,target_pc):
    '''
    find correspondences between source and target arrays.
    returns indices of corresponding points in source array
    '''
    #shape_source=source.shape[0]
    #shape_target=target.shape[0]
    #if shape_source<shape_target:
        #target=target[:shape_source,:]
    #else:
        #source=source[:shape_target,:]
    print('printing shape now')
    print(f'shape of source is{source_pc.shape}')
    print(f'shape of target is{target_pc.shape}')
    # new
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(source_pc)
    #neigh.fit(target_pc)
    distances, indices = neigh.kneighbors(target_pc, return_distance=True)
    #distances, indices = neigh.kneighbors(source_pc, return_distance=True)
    print(f'the len of indices is {len(indices)}')
    source_pc_out=source_pc[indices].reshape((len(indices),3))
    print(f'new source shape is {source_pc_out.shape}')
    return distances.ravel(), indices.ravel(),source_pc_out,target_pc

def ICP(source_pcd,target_pcd):
    
    no_iterations=50
    prev_error=0
    new_source=source_pcd

    for n in range(no_iterations):
        print(f'iteration{n}')
        dist,ind,source_kn,target_kn=find_correspondences(new_source,target_pcd)
        print(f'shape of target is{target_kn.shape}')
        print(f'shape of source is {source_kn.shape}')
        Transf,rot,trans,c_source,c_target,mean_tar=Transformation(ind,source_kn,target_kn)
        n_source=(np.matmul(rot,source_kn.T)).T+trans.reshape((3,))
        #n_source=(np.matmul(rot,source_kn.T)).T+c_target.reshape((3,))
        #n_source=(np.matmul(rot,c_source.T)).T
        print(f'shape of new source is {n_source.shape}')
        #new_source=new_source.T
        #mse_error=mean_squared_error(n_source,target_kn)
        mse_error=mean_squared_error(n_source,target_kn)
        mae_error=mean_absolute_error(n_source,target_kn)
        print(f'iteration {n}: MSE error :{mse_error}')
        print(f'iteration {n}: MAE error :{mae_error}')
        print(f'iteration {n}: mean distance is:{np.mean(dist)}')
        #error=np.mean(d)
        if (np.abs(mse_error-prev_error)<0.0001):
            break
        prev_error=mse_error
        new_source=n_source
    
    return Transf

demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
initial_transfomration=np.asarray([[0.862 ,0.011,-0.507,0.5],[-0.139,0.967,-0.215,0.7],
                [0.487,0.255,0.835,-1.4],[0,0,0,1]])
source=source.transform(initial_transfomration)
target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
# converting the shape of source and target point cloud
source_array=np.asarray(source.points)
target_array=np.asarray(target.points)
transformation=ICP(source_array,target_array)
file=np.save('new_transf',transformation)

#new=np.load(r'C:\Users\kc\Desktop\.venv\Perception\Assignment2\Questions\Question 2\new_transf.npy')
source.transform(transformation)
o3d.visualization.draw_geometries([source, target],
zoom=0.4459,
front=[0.9288, -0.2951, -0.2242],
lookat=[1.6784, 2.0612, 1.4451],
up=[-0.3402, -0.9189, -0.1996])










            
    
















