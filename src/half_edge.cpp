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

    // from description derivation, we get the optimal contraction coordinate
    for (auto neighbor_vertex : v1->neighbor_vertices()) {
        this->verts_contract_pos += neighbor_vertex->pos;
    }
    for (auto neighbor_vertex : v2->neighbor_vertices()) {
        this->verts_contract_pos += neighbor_vertex->pos;
    }
    this->verts_contract_pos -= v1->pos + v2->pos;

    this->verts_contract_pos /= (v1->neighbor_vertices().size() + v2->neighbor_vertices().size() - 2);
    
    auto v_star = this->verts_contract_pos;

    // [v*Tv*; -2v*; 1]
    Eigen::VectorXf v5f = Eigen::VectorXf(5);
    v5f << v_star.dot(v_star), -2 * v_star, 1;

    // QEM = v*.T * Q * v*
    this->qem = q_star.dot(v5f);
}

// void Edge::compute_contraction() {
//     this->verts_contract_pos = Eigen::Vector3f(0, 0, 0);
//     this->qem = 0;

//     // v1, v2
//     std::shared_ptr<Vertex> v1 = this->he->vertex;
//     std::shared_ptr<Vertex> v2 = this->he->twin->vertex;

//     // v1, v2's coefficient with size 5
//     Eigen::VectorXf q1 = v1->qem_coff;
//     Eigen::VectorXf q2 = v2->qem_coff;

//     // q* = q1 + q2 //s refer: https://edstem.org/au/courses/15623/discussion/1868886
//     /*
//         Suppose we have vertices v1 and v2, each associated with quadratic coefficient q1 and q2, so that we can come up with E(v1)=q1v1 and E(v2)=q2v2, right?  Similarly, the combined quadratic error for the union of v1, v2 can be written as E(v*)=q*v*. In this case, we must derive a new matrix coefficient q* which approximates the error and we have chosen to use the simple additive rule q* = q1 + q2.
//         Be aware that the minimizer is the point where derivative is zero, therefore how you should compute the optimal contraction coordinate is to first compute q*, and then taking its derivative and set it to zero.
//         Haorui
//     */
//     Eigen::VectorXf q_star = q1 + q2;

//     // after derivation, we get the optimal contraction coordinate
//     this->verts_contract_pos = -q_star.segment(1, 3) / (2 * q_star(4));  // TODO wrong derivation
//     auto v_star = this->verts_contract_pos;

//     // [v*Tv*; -2v*; 1]
//     Eigen::VectorXf v5f = Eigen::VectorXf(5);
//     v5f << v_star.dot(v_star), -2 * v_star, 1;

//     // QEM = v*.T * Q * v*
//     this->qem = q_star.dot(v5f);
// }


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
    // 10 half edges as sketch
    std::shared_ptr<HalfEdge> he1 = this->he;
    std::shared_ptr<HalfEdge> he2 = he1->twin;
    std::shared_ptr<HalfEdge> he3 = he1->next;
    std::shared_ptr<HalfEdge> he4 = he2->next;
    std::shared_ptr<HalfEdge> he5 = he1->next->next;
    std::shared_ptr<HalfEdge> he6 = he2->next->next;
    std::shared_ptr<HalfEdge> he7 = he3->twin;
    std::shared_ptr<HalfEdge> he8 = he4->twin;
    std::shared_ptr<HalfEdge> he9 = he5->twin;
    std::shared_ptr<HalfEdge> he10 = he6->twin;

    // v1, v2, v3, v4
    std::shared_ptr<Vertex> v1 = he1->vertex;
    std::shared_ptr<Vertex> v2 = he2->vertex;
    std::shared_ptr<Vertex> v3 = he1->next->next->vertex;
    std::shared_ptr<Vertex> v4 = he2->next->next->vertex;

    // e1, e2, e3, e4, e5
    std::shared_ptr<Edge> e1 = he1->edge;
    std::shared_ptr<Edge> e2 = he3->edge;
    std::shared_ptr<Edge> e3 = he5->edge;
    std::shared_ptr<Edge> e4 = he4->edge;
    std::shared_ptr<Edge> e5 = he6->edge;


    // f1, f2
    std::shared_ptr<Face> f1 = he1->face;
    std::shared_ptr<Face> f2 = he2->face;

    // debug:
    // auto neighbor_vertices_1 = v1->neighbor_vertices();
    // auto neighbor_vertices_2 = v2->neighbor_vertices();
    // assert(neighbor_vertices_1.size() >= 3);  // why sometimes 2?
    // assert(neighbor_vertices_2.size() >= 3);


    // (i) Moves the vertex v1 to the new position v*, remember to update all corresponding attributes,
    
    // a.1. update the position of v1
    v1->pos = this->verts_contract_pos;
    
    // a.2. v1->he need to be updated if the halfedge's edge is this edge to be collapsed
    // update the he of verteices
    v1->he = he9;
    // v2->he = nullptr;
    v3->he = he7;
    v4->he = he8;

    // a.3. update the qem coefficient after collapasing  
    v1->qem_coff += v2->qem_coff; 
    
    // (ii) Connects all incident edges of v1 and v2 to v*, and remove the vertex v2,

    // b.1. update the halfedge of v2: next, twin (unchange), face, vertex, edge (unchange), exists (after), id (unchange)
    // assume all faces are triangle
    // b.1.1. update the boundary halfedges of v1 and v2, and the edge of the boundary halfedges
    
    // update the boundary twin hf
    he9->twin = he7;
    he7->twin = he9;
    he10->twin = he8;
    he8->twin = he10;

    // update the edge of the boundary halfedges
    he7->edge = he9->edge;
    he10->edge = he8->edge; 

    // update the he of the edge    
    e3->he = he9; // he9 or he7
    e4->he = he8; // he8 or he10    
    // after the above, the boundary halfedges of v1 and v2 are connected to each other


    // b.2. update the vertex of the halfedge of v2
    std::shared_ptr<HalfEdge> he_iter = he9;  // start from the he9
    do {
        he_iter->vertex = v1;
        he_iter = he_iter->twin->next;
    } while (he_iter != he8->next);  // he8->next is where the he_iter of v1 starts

    // b.3. remove the vertex v2 by setting exists to false
    v2->exists = false;

    // (iii) All faces, half edges, and edges associated with this collapse edge will be removed.

    // c.1. remove all (two) faces
    f1->exists = false;  
    f2->exists = false;

    // c.2. remove all (six) half edges
    he1->exists = false;
    he2->exists = false;
    he3->exists = false;
    he4->exists = false;
    he5->exists = false;
    he6->exists = false;

    // c.3. remove all (three) edges
    e1->exists = false;
    e2->exists = false;
    e5->exists = false;

    // v1->compute_qem_coeff();
}