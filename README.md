# ICP
Custom function to implement ICP algorithm without using available APIs 

# Process
* Find correspondences between source and target point cloud using nearest neighbor method
* Find the transformation based on the correspondences. We first need to demean the point clouds (subtract point
clouds from centroid).The dot product of the demeaned point cloud generates the covariance matrix.
* I followed the manual work of peroforming the dot product of the demeaned source and target point cloud. There
was another alternative of using the orthogonalprocrustes from scipy however to follow the instructions of minimizing
the use of the API in the critical parts of the program, I performed the dot product and continued with the Singular
Value Decomposition.
* Performing the singular value decomposition on the H matrix, and setting R as the product of U (V.T), we can
find the rotation matrix. The translation matrix can be obtained by subtracting the mean target from the mean source transformed by the obtained rotation matrix.
* Based on the generated rotation and translation, we will move the source point cloud and compute the error between
the source and target point clouds
* Setting the algorithm for initial iterations=50 and specifying the stopping conditions that if the absolute difference
berween the previous and current error failed to improve, we can break as long as there is reduction in the mean
square error.

# Results
![image](https://user-images.githubusercontent.com/69100847/210170274-fe970ee2-7180-48f6-afa7-625dc2872b35.png)
