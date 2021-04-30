from scipy.spatial.distance import euclidean

class Graph(object):

    def __init__(self, graph_dict=None):
        """ initializes a graph object 
            If no dictionary or None is given, 
            an empty dictionary will be used
        """
        if graph_dict is None:
            graph_dict = {}
        self._graph_dict = graph_dict

    def edges(self, vertice):
        """ returns a list of all the edges of a vertice"""
        return self._graph_dict[vertice]
        
    def all_vertices(self):
        """ returns the vertices of the graph as a set """
        return set(self._graph_dict.keys())

    def all_edges(self):
        """ returns the edges of the graph """
        return self.__generate_edges()

    def add_vertex(self, vertex, min_dist_lim = 0.2):
        """ If the vertex "vertex" is not in self._graph_dict and its closest vertex is at least min_dist_lim far, 
            a key "vertex" with an empty list as a value is added to the dictionary.
            Otherwise nothing has to be done.
            Returns True if added, False otherwise.
        """
        min_dist = 999
        for v in self._graph_dict:
            dist = euclidean(vertex, v)
            if dist < min_dist:
                min_dist = dist

        if vertex not in self._graph_dict and min_dist > min_dist_lim:
            self._graph_dict[vertex] = []
            return True
        else:
            return False

    def add_edge(self, edge):
        """ adds edge between two vertices. edge can be either list or tuple """
        edge = set(edge)
        vertex1, vertex2 = tuple(edge)
        for x, y in [(vertex1, vertex2), (vertex2, vertex1)]:
            if x in self._graph_dict:
                self._graph_dict[x].append(y)
            else:
                self._graph_dict[x] = [y]

    def get_shortest_path(self, start, goal):
        """ returns None if there is no shortest path
            empty if there start == goal
            shortest path in order as list
        """
        explored = []
        
        # Queue for traversing the
        # graph in the BFS
        queue = [[start]]
        
        # If the desired node is
        # reached
        if start == goal:
            return []
        
        # Loop to traverse the graph
        # with the help of the queue
        while queue:
            path = queue.pop(0)
            node = path[-1]
            
            # Condition to check if the
            # current node is not visited
            if node not in explored:
                neighbours = self._graph_dict[node]
                
                # Loop to iterate over the
                # neighbours of the node
                for neighbour in neighbours:
                    new_path = list(path)
                    new_path.append(neighbour)
                    queue.append(new_path)
                    
                    # Condition to check if the
                    # neighbour node is the goal
                    if neighbour == goal:
                        return new_path
                explored.append(node)
    
        return None

    def __generate_edges(self):
        """ A static method generating the edges of the 
            graph "graph". Edges are represented as sets 
            with one (a loop back to the vertex) or two 
            vertices 
        """
        edges = []
        for vertex in self._graph_dict:
            for neighbour in self._graph_dict[vertex]:
                if {neighbour, vertex} not in edges:
                    edges.append({vertex, neighbour})
        return edges
    
    def __iter__(self):
        self._iter_obj = iter(self._graph_dict)
        return self._iter_obj
    
    def __next__(self):
        """ allows us to iterate over the vertices """
        return next(self._iter_obj)

    def __str__(self):
        res = "vertices: "
        for k in self._graph_dict:
            res += str(k) + " "
        #res += "\nedges: "
        #for edge in self.__generate_edges():
        #    res += str(edge) + " "
        return res