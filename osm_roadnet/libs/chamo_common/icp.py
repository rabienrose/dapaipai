import numpy as np
from scipy.spatial.distance import cdist

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
      R: 3x3 rotation matrix
      t: 3x1 column vector
    '''

    assert len(A) == len(B)

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
#    print(centroid_A)
#    print(centroid_B)
#    print(np.dot(R,centroid_A.T).T)
#    a=a+1
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    all_dists = cdist(src, dst, 'euclidean')
    indices = all_dists.argmin(axis=1)
    distances = all_dists[np.arange(all_dists.shape[0]), indices]
    return distances, indices

def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.0001):
    '''
    The Iterative Closest Point method
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_pose: 4x4 homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation
        distances: Euclidean distances (errors) of the nearest neighbor
    '''

    # make points homogeneous, copy them so as to maintain the originals
    src = np.ones((4,A.shape[0]))
    dst = np.ones((4,B.shape[0]))
    src[0:3,:] = np.copy(A.T)
    dst[0:3,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0
    T_total=np.identity(4)
    for i in range(max_iterations):
        # find the nearest neighbours between the current source and destination points
        distances, indices = nearest_neighbor(src[0:3,:].T, dst[0:3,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[0:3,:].T, dst[0:3,indices].T)
        
        # update the current source
        src = np.dot(T, src)
        T_total=np.dot(T_total,T)
        # check error
        mean_error = np.sum(distances) / distances.size
        if abs(prev_error-mean_error) < tolerance:
            break
        prev_error = mean_error
    
    T,_,_ = best_fit_transform(A, src[0:3,:].T)
#    src = np.ones((4,A.shape[0]))
#    src[0:3,:] = np.copy(A.T)
#    print(T_total)
#    print(T )
#    src = np.dot(T, src)
#    y=np.linalg.norm(src[0:3,:]-dst[0:3,indices], axis=0)
#    print(np.sum(y) / y.size)
    return T, distances, indices
    
def icp_mindist(A, B, init_pose=None, max_iterations=20, tolerance=0.0001):
    '''
    The Iterative Closest Point method
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_pose: 4x4 homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation
        distances: Euclidean distances (errors) of the nearest neighbor
    '''

    src = np.ones((4,A.shape[0]))
    dst = np.ones((4,B.shape[0]))
    src[0:3,:] = np.copy(A.T)
    dst[0:3,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0
    T_total=np.identity(4)
    match_count=0
    for i in range(max_iterations):
        # find the nearest neighbours between the current source and destination points
        distances, indices = nearest_neighbor(src[0:3,:].T, dst[0:3,:].T)
        count=0
        indices_filter=[]
        for dist in distances:
            if dist<3:
                indices_filter.append(count)
            count=count+1
        # compute the transformation between the current source and nearest destination points
        match_count=len(indices_filter)
        #print(match_count)
        if len(indices_filter)<20:
            break;
        
        src_filter = np.ones((3,len(indices_filter)), src.dtype)
        dst_filter = np.ones((3,len(indices_filter)), src.dtype)
        count=0
        #print(str(len(indices_filter))+"/"+str(len(indices)))
        for ind in indices_filter:
            src_filter[:, count]=src[0:3,ind]
            dst_filter[:, count]=dst[0:3,indices[ind]]
            count=count+1
            
#        print(src_filter.T)
#        print(dst_filter.T)
#        a=a+1
        T,_,_ = best_fit_transform(src_filter.T, dst_filter.T)
    
        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.sum(distances[indices_filter]) / match_count
        #print(mean_error)
        if abs(prev_error-mean_error) < tolerance:
            break
        prev_error = mean_error
    T,_,_ = best_fit_transform(A, src[0:3,:].T)
    return T, distances, match_count
    
    
