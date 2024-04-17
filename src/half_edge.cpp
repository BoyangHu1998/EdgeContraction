#include <include/half_edge.hpp>
#include <Eigen/Dense>

/*
#################################################################################
#                       Vertex-related Helper Functions                         #
#################################################################################
*/

// Optinal TODO: Iterate through all neighbour vertices around the vertex
// Helpful when you implement the average degree computation of the mesh
std::vector<std::shared_ptr<Vertex>> Vertex::neighbor_vertices() {
    std::vector<std::shared_ptr<Vertex>> neighborhood;

    if (he == nullptr || he->twin == nullptr) {
        return neighborhood;
    }

    auto he_iter = he;
    do {
        neighborhood.push_back(he_iter->twin->vertex);
        he_iter = he_iter->twin->next;
    } while (he != he_iter);

    return neighborhood; 
}


// TODO: Iterate through all half edges pointing away from the vertex
std::vector<std::shared_ptr<HalfEdge>> Vertex::neighbor_half_edges() {  // w6 pg99
    std::vector<std::shared_ptr<HalfEdge>> neighborhood;

    if (he == nullptr || he->twin == nullptr) {
        return neighborhood;
    }

    auto he_iter = he;
    do {
        assert(he_iter->twin != nullptr);
        neighborhood.push_back(he_iter);
        he_iter = he_iter->twin->next; 
    } while (he != he_iter);

    return neighborhood;
}


// TODO: Computate quadratic error metrics coefficient, which is a 5-d vector associated with each vertex
/*
    HINT:
        Please refer to homework description about how to calculate each element in the vector.
        The final results is stored in class variable "this->qem_coff"
*/
void Vertex::compute_qem_coeff() {
    this->qem_coff = Eigen::VectorXf(5);

    // q = [n; sum(vi); sum(vi.T * vi)]
    auto neighbor_vertices = this->neighbor_vertices();
    int n = neighbor_vertices.size();
    Eigen::Vector3f sum_v(0, 0, 0);
    float sum_vtv = 0;

    for (auto v : neighbor_vertices) {
        sum_v += v->pos;
        sum_vtv += v->pos.dot(v->pos);
    }

    this->qem_coff(0) = n;
    this->qem_coff.segment(1, 3) = sum_v;
    this->qem_coff(4) = sum_vtv;
    // debug:  (float[5])*((this->qem_coff).data())
}


/*
#################################################################################
#                         Face-related Helper Functions                         #
#################################################################################
*/

// TODO: Iterate through all member vertices of the face
std::vector<std::shared_ptr<Vertex>> Face::vertices() {  // w6 pg107
    std::vector<std::shared_ptr<Vertex>> member_vertices;

    if (he == nullptr || !he->exists) {
        return member_vertices;
    }

    auto he_iter = he;
    do {
        member_vertices.push_back(he_iter->vertex);
        he_iter = he_iter->next;
    } while (he != he_iter);

    return member_vertices;
}


// TODO: implement this function to compute the area of the triangular face
float Face::get_area(){
    float area;

    Eigen::Vector3f v1 = he->vertex->pos;
    Eigen::Vector3f v2 = he->next->vertex->pos;
    Eigen::Vector3f v3 = he->next->next->vertex->pos;

    Eigen::Vector3f e1 = v2 - v1;
    Eigen::Vector3f e2 = v3 - v1;

    area = 0.5 * (e1.cross(e2)).norm();

    return area;
}

// TODO: implement this function to compute the signed volume of the triangular face
// reference: http://chenlab.ece.cornell.edu/Publication/Cha/icip01_Cha.pdf eq.(5)
float Face::get_signed_volume(){
    float volume;

    Eigen::Vector3f v1 = he->vertex->pos;
    Eigen::Vector3f v2 = he->next->vertex->pos;
    Eigen::Vector3f v3 = he->next->next->vertex->pos;

    // CHECK: the paper
    volume = (1.0/6.0) * (v1.dot(v2.cross(v3))); // 1/3 * Area * h // Area = 1/2 * |v1 x v2| // h = |v3 . (v1 x v2)| / |v1 x v2|

    return volume;
}


/*
#################################################################################
#                         Edge-related Helper Functions                         #
#################################################################################
*/

/*
    TODO: 
        Compute the contraction information for the edge (v1, v2), which will be used later to perform edge collapse
            (i) The optimal contraction target v* (How to compute? CHECK)
            (ii) The quadratic error metrics QEM, which will become the cost of contracting this edge
        The final results is stored in class variable "this->verts_contract_pos" and "this->qem"
    Please refer to homework description for more details
*/
void Edge::compute_contraction() {
    this->verts_contract_pos = Eigen::Vector3f(0, 0, 0);
    this->qem = 0;

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(3, 3);
    Eigen::VectorXf B = Eigen::VectorXf::Zero(3);

    // v1, v2
    std::shared_ptr<Vertex> v1 = this->he->vertex;
    std::shared_ptr<Vertex> v2 = this->he->twin->vertex;

    // v1, v2's coefficient with size 5
    Eigen::VectorXf q1 = v1->qem_coff;
    Eigen::VectorXf q2 = v2->qem_coff;

    // q* = q1 + q2 //s refer: https://edstem.org/au/courses/15623/discussion/1868886
    /*
        Suppose we have vertices v1 and v2, each associated with quadratic coefficient q1 and q2, so that we can come up with E(v1)=q1v1 and E(v2)=q2v2, right?  Similarly, the combined quadratic error for the union of v1, v2 can be written as E(v*)=q*v*. In this case, we must derive a new matrix coefficient q* which approximates the error and we have chosen to use the simple additive rule q* = q1 + q2.
        Be aware that the minimizer is the point where derivative is zero, therefore how you should compute the optimal contraction coordinate is to first compute q*, and then taking its derivative and set it to zero.
        Haorui
    */
    Eigen::VectorXf q_star = q1 + q2;

    // after derivation, we get the optimal contraction coordinate
    this->verts_contract_pos = -q_star.segment(1, 3) / (2 * q_star(4));

    // [v*Tv*; -2v*; 1]
    Eigen::VectorXf v5f = Eigen::VectorXf(5);
    v5f << this->verts_contract_pos.dot(this->verts_contract_pos), -2 * this->verts_contract_pos, 1;

    // QEM = v*.T * Q * v*
    this->qem = q_star.dot(v5f);
}


/*
    TODO: 
        Perform edge contraction functionality, which we write as (v1 ,v2) → v*, 
            (i) Moves the vertex v1 to the new position v*, remember to update all corresponding attributes,
            (ii) Connects all incident edges of v1 and v2 to v*, and remove the vertex v2,
            (iii) All faces, half edges, and edges associated with this collapse edge will be removed.
    HINT: 
        (i) Pointer reassignments
        (ii) When you want to remove mesh components, simply set their "exists" attribute to False
    Please refer to homework description for more details
*/
void Edge::edge_contraction() {
    // v1, v2
    std::shared_ptr<Vertex> v1 = this->he->vertex;
    std::shared_ptr<Vertex> v2 = this->he->twin->vertex;

    // (i) Moves the vertex v1 to the new position v*, remember to update all corresponding attributes,

    // 1. update the position of v1
    v1->pos = this->verts_contract_pos;
    
    // 2. v1->he need to be updated if the halfedge's edge is this edge
    if (v1->he->edge.get() == this) {  // using .get() to get the raw pointer from shared_ptr
        v1->he = v1->he->twin->next;
    }

    // 3. update the qem coefficient
    v1->compute_qem_coeff(); 

    // 4. exists and id doesn't need to be updated

    
    // (ii) Connects all incident edges of v1 and v2 to v*, and remove the vertex v2,

    // 1. update the halfedge of v2: next, twin (unchange), face, vertex, edge (unchange), exists (after), id (unchange)
    // 1.1. find the previous halfedge of v2
    
    // 2. update the halfedge of v1
    // 2.1. find the previous halfedge of v1s

    // 3. update the vertex of the halfedge of v2


    // update the next halfedge of the previous halfedge of v2

    // update the next halfedge of the previous halfedge of v1


    // 4. remove the vertex v2 by setting exists to false
    v2->exists = false;


    // (iii) All faces, half edges, and edges associated with this collapse edge will be removed.

    // 1. remove all (two) faces
    this->he->face->exists = false;  
    this->he->twin->face->exists = false;

    // 2. remove all (two) half edges
    this->he->exists = false; 
    this->he->twin->exists = false;

    // 3. remove the edge
    this->exists = false;
}