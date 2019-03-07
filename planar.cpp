#include <iostream>
#include <vector>
#include <set>
#include <algorithm>

using namespace std;

class Face;

class ListGraph {
public:
    ~ListGraph() = default;
    explicit ListGraph(int vertNumber);
    void AddEdge(int from, int to);
    int VerticesCount() const;
    bool areAdjacent(int v1, int v2) const { return find(toList[v1].begin(), toList[v1].end(), v2) != toList[v1].end(); }
    bool isolated(int v) const { return toList[v].empty() && fromList[v].empty(); }
    int GetNextVertices(int vertex, vector<int>& vertices) const;
    int GetPrevVertices(int vertex, vector<int>& vertices) const;

private:
    vector<vector<int>> toList;
    vector<vector<int>> fromList;
};

ListGraph::ListGraph(int vertNumber) {
    for( int i = 0; i < vertNumber; ++i ) {
        toList.emplace_back(vector<int>());
        fromList.emplace_back(vector<int>());
    }
}

void ListGraph::AddEdge(int from, int to) {
    toList[from].push_back(to);
}

int ListGraph::GetNextVertices(int vertex, vector<int> &vertices) const {
    for( int n : toList[vertex] )
        vertices.push_back(n);
    return static_cast<int>(toList[vertex].size());
}

int ListGraph::GetPrevVertices(int vertex, vector<int> &vertices) const {
    for( int n : fromList[vertex] )
        vertices.push_back(n);
    return static_cast<int>(fromList[vertex].size());
}

int ListGraph::VerticesCount() const {
    return static_cast<int>(toList.size());
}

class Segment : public ListGraph {
public:
    explicit Segment(int vertNumber) : ListGraph(vertNumber) {}
    Segment(int vertNumber, set<int> contacts) : ListGraph(vertNumber), contactVertices(move(contacts)) {}
    void addContacts(const set<int>& contacts) { contactVertices.insert(contacts.begin(), contacts.end()); }
    int fitsIntoFaces(const vector<Face*>&, vector<Face*>&) const;
    int fitsIntoFaces(const vector<Face*>&) const;
    void getContacts(set<int>& contacts) const { contacts = contactVertices; }
    bool isContact(int vertex) const { return find(contactVertices.begin(), contactVertices.end(), vertex) != contactVertices.end(); }
private:
    set<int> contactVertices;
};

class Face {
public:
    explicit Face(vector<int> vertices) : cycle(move(vertices)) {}
    Face(Face*, vector<int> vertices);
    bool contains(const Segment*) const;
private:
    vector<int> cycle; // Face lies at the right side
};

int Segment::fitsIntoFaces(const vector<Face*>& faces, vector<Face*>& fits) const {
    int counter = 0;
    for( Face *face : faces ) {
        if( face->contains(this) ) {
            fits.push_back(face);
            ++counter;
        }
    }

    return counter;
}

int Segment::fitsIntoFaces(const vector<Face*>& faces) const {
    int counter = 0;
    for( Face *face : faces ) {
        if( face->contains(this) ) {
            ++counter;
        }
    }
    return counter;
}

bool Face::contains(const Segment* segment) const {
    set<int> contacts;
    segment->getContacts(contacts);
    for( int contact : contacts ) {
        if( find(cycle.begin(), cycle.end(), contact ) == cycle.end() )
            return false;
    }
    return true;
}

Face::Face(Face* original, vector<int> vertices) { // Copy because it might change
    int firstContact = -1, lastContact = -1;
    for( int i = 0; i < original->cycle.size(); ++i ) {
        if( original->cycle[i] == vertices[0] )
            firstContact = i;
        else if( original->cycle[i] == vertices[vertices.size() - 1])
            lastContact = i;
    }
    if( firstContact > lastContact )
        swap(firstContact, lastContact);
    if( vertices[0] == original->cycle[lastContact] )
        reverse(vertices.begin(), vertices.end());

    // Go from 0 to 1st contact, insert vertices, then add 2nd - last
    // Then go from 1st to 2nd, add reversed vertices
    vector<int> tmpCycle(original->cycle.begin() + firstContact + 1, original->cycle.begin() + lastContact);
    original->cycle.erase(original->cycle.begin() + firstContact + 1, original->cycle.begin() + lastContact);
    original->cycle.insert(original->cycle.begin() + firstContact + 1, vertices.begin() + 1, vertices.end() - 1);

    reverse(vertices.begin(), vertices.end());
    vertices.insert(vertices.end(), tmpCycle.begin(), tmpCycle.end());
    this->cycle = vector<int>(vertices);
}

int findMinSegment(const vector<Segment*>& segments, const vector<Face*>& faces) {
    if( segments.empty() )
        return -1;
    int least = -1;
    int minValue = INT32_MAX;
    for( int i = 0; i < segments.size(); ++i ) {
        int curr = segments[i]->fitsIntoFaces(faces);
        if( curr < minValue ) {
            least = i;
            minValue = curr;
        }
    }

    return least;
}

void dfsForSegments(const ListGraph& graph, int vertex, const ListGraph& Gplane, vector<char>& visited, Segment *seg, set<int>& contacts) {
    vector<int> next;
    visited[vertex] = 1;
    graph.GetNextVertices(vertex, next);
    for( int n : next ) {
        if( !Gplane.isolated(n) ) {// If n in Gplane
            contacts.insert(n);
            seg->AddEdge(n, vertex);
        }
        else if( !visited[n] ) {
            dfsForSegments(graph, n, Gplane, visited, seg, contacts);
        }
        seg->AddEdge(vertex, n);
    }
}

Segment *findSegment(const ListGraph& graph, int vertex, const ListGraph& Gplane) {
    // Traverse using DFS, add edges into the segment
    vector<char> visited(graph.VerticesCount(), 0);
    auto seg = new Segment(graph.VerticesCount());
    set<int> contacts;
    dfsForSegments(graph, vertex, Gplane, visited, seg, contacts);
    seg->addContacts(contacts);

    return seg;
}

void splitIntoSegments(const ListGraph& graph, const vector<int>& contacts, vector<Segment*>& segments, ListGraph& Gplane) {
    vector<int> next;
    set<pair<int, int>> usedContactEdges;
    for (int vertex : contacts) {
        graph.GetNextVertices(vertex, next);
        for (int n : next) {
            // First check Gplane, then - if visited
            if (!Gplane.areAdjacent(n, vertex) && usedContactEdges.find(make_pair(vertex, n)) == usedContactEdges.end()) {
                Segment *newSegment = nullptr;
                if( !Gplane.isolated(n) ) { // If already laid
                    newSegment = new Segment(max(n, vertex) + 1, set<int> {vertex, n});
                    newSegment->AddEdge(vertex, n);
                    newSegment->AddEdge(n, vertex);
                    usedContactEdges.insert(make_pair(vertex, n));
                    usedContactEdges.insert(make_pair(n, vertex));
                }
                else {
                    newSegment = findSegment(graph, n, Gplane);
                    // Mark used edges
                    if (newSegment != nullptr) {
                        set<int> segContacts;
                        vector<int> segNext;
                        newSegment->getContacts(segContacts);
                        for (int v : segContacts) {
                            newSegment->GetNextVertices(v, segNext);
                            for (int segN : segNext) {
                                usedContactEdges.insert(make_pair(v, segN));
                                usedContactEdges.insert(make_pair(segN, v));
                            }
                            segNext.clear();
                        }
                    }
                }
                segments.push_back(newSegment);
            }
            next.clear();
        }
    }
}

bool dfs(const Segment& seg, vector<int>& path, vector<char>& visited, int vertex) {
    vector<int> next;
    visited[vertex] = 1;
    seg.GetNextVertices(vertex, next);
    for( int n : next ) {
        if( !visited[n] ) {
            if( seg.isContact(n) ) {
                path.push_back(n);
                path.push_back(vertex);
                return true;
            }
            else {
                bool res = dfs(seg, path, visited, n);
                if( res ) {
                    path.push_back(vertex);
                    return res;
                } else {
                    continue;
                }
            }
        }
    }
    return false;
}

void findPath(const Segment& seg, vector<int>& path) {
    vector<char> visited(seg.VerticesCount(), 0);
    set<int> contacts;
    seg.getContacts(contacts);
    int vertex = *contacts.begin();
    dfs(seg, path, visited, vertex);
}

int findCycle(const ListGraph& graph, vector<int>& contacts, vector<char>& color, int parent = -1, int vertex = 0) {
    color[vertex] = 'g';
    vector<int> next;
    graph.GetNextVertices(vertex, next);
    for( int n : next ) {
        if( color[n] == 'w' ) {
            int res = findCycle(graph, contacts, color, vertex, n);
            if( res != vertex && res != -1) {
                contacts.push_back(vertex);
                return res;
            } else return -1;
        }
        else if( color[n] == 'g' && parent != n ) {
            contacts.push_back(n);
            contacts.push_back(vertex);
            return n;
        }
    }
    color[vertex] = 'b';

    return 1;
}

bool is_planar(const ListGraph& graph) { // Assume there are no bridges
    ListGraph Gplane(graph.VerticesCount());
    vector<int> contacts;
    vector<Segment*> segments;
    vector<Face*> faces;

    vector<char> color(static_cast<unsigned>(graph.VerticesCount()), 'w');
    findCycle(graph, contacts, color);
    for( int i = 0; i < contacts.size(); ++i ) {
        Gplane.AddEdge(contacts[i], contacts[(i + 1) % contacts.size()]);
        Gplane.AddEdge(contacts[(i + 1) % contacts.size()], contacts[i]);
    }
    faces.emplace_back(new Face(contacts));
    reverse(contacts.begin(), contacts.end());
    faces.emplace_back(new Face(contacts));

    splitIntoSegments(graph, contacts, segments, Gplane);

    while( !segments.empty() ) {
        int tmp = findMinSegment(segments, faces);
        Segment *least = segments[tmp];
        segments.erase(segments.begin() + tmp);
        vector<Face*> faceForSplit;
        if (least->fitsIntoFaces(faces, faceForSplit) == 0) {
            for( Segment *seg : segments )
                delete seg;
            for( Face *face : faces )
                delete face;
            delete least;

            return false;
        }
        // Find a path from one contact vertex to another
        // Then split the face, adding this path
        vector<int> path;
        findPath(*least, path);
        for( int i = 0; i + 1 < path.size(); ++i ) {
            Gplane.AddEdge(path[i], path[i + 1]);
            Gplane.AddEdge(path[i + 1], path[i]);
        }
        Face *newFace = new Face(faceForSplit[0], path);
        faces.push_back(newFace); // Split faces
        if( path.size() != least->VerticesCount() ) {
            splitIntoSegments(*least, path, segments, Gplane);
        }
        delete least;
    }

    for( Segment *seg : segments )
        delete seg;
    for( Face *face : faces )
        delete face;

    return true;
}

int main() {
    int vertices = 0, edges = 0, from = 0, to = 0;
    cin >> vertices >> edges;
    ListGraph graph(vertices);
    for( int i = 0; i < edges; ++i ) {
        cin >> from >> to;
        graph.AddEdge(from, to);
        graph.AddEdge(to, from);
    }

    cout << (is_planar(graph) ? "YES" : "NO");

    return 0;
}
