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

    if (he == nullptr || !he->exists) {
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

    if (he == nullptr || !he->exists) {
        return neighborhood;
    }

    auto he_iter = he;
    do {
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
            (i) The optimal contraction target v*
            (ii) The quadratic error metrics QEM, which will become the cost of contracting this edge
        The final results is stored in class variable "this->verts_contract_pos" and "this->qem"
    Please refer to homework description for more details
*/
void Edge::compute_contraction() {
    this->verts_contract_pos = Eigen::Vector3f(0, 0, 0);
    this->qem = 0;
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
    
}