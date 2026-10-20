import heapq
from algorithms.pathfinding.static_dijkstra import StaticDijkstra
from core.registry import AlgorithmRegistry

class DStarLite(StaticDijkstra):
    """
    Implementasi Algoritma D* Lite (Koenig & Likhachev, 2002).
    Mewarisi StaticDijkstra untuk memanfaatkan logging dan struktur dasar graf.
    """

    def __init__(self, graph, start_node, goal_node):
        super().__init__(graph, start_node, goal_node)
        
        # Struktur Data D* Lite
        self.g = {}          # Estimasi jarak terpendek dari node ke goal
        self.rhs = {}        # Nilai lookahead 1-step
        self.k_m = 0.0       # Key modifier untuk pergerakan start_node
        self.U = []          # Priority Queue (min-heap): list of (key, node)
        self.open_set = set() # Penanda node yang ada di dalam Priority Queue U

        # Inisialisasi awal nilai g dan rhs untuk semua simpul
        for node in self.graph.nodes:
            self.g[node] = float('inf')
            self.rhs[node] = float('inf')

        # Kondisi awal pencarian terbalik (backward search dari goal ke start)
        self.rhs[self.goal_node] = 0.0
        self._insert_queue(self.goal_node, self._calculate_key(self.goal_node))

    # =========================================================================
    # FUNGSI BANTUAN PRIORITY QUEUE & KEY
    # =========================================================================

    def _heuristic(self, node_a, node_b):
        """
        Fungsi heuristik h(a, b). 
        Dapat menggunakan jarak Euclid/Manhattan atau 0.0 (Dijkstra-style).
        """
        return 0.0

    def _calculate_key(self, node):
        """
        Menghitung nilai kunci [k1, k2] untuk prioritas di antrean U.
        """
        min_g_rhs = min(self.g[node], self.rhs[node])
        k1 = min_g_rhs + self._heuristic(self.start_node, node) + self.k_m
        k2 = min_g_rhs
        return (k1, k2)

    def _insert_queue(self, node, key):
        heapq.heappush(self.U, (key, node))
        self.open_set.add(node)

    def _remove_queue(self, node):
        if node in self.open_set:
            self.open_set.remove(node)
            # Rebuild heap tanpa node tersebut
            self.U = [(k, n) for k, n in self.U if n != node]
            heapq.heapify(self.U)

    # =========================================================================
    # ALGORITMA INTI D* LITE
    # =========================================================================

    def _update_vertex(self, u):
        """
        Memperbarui status konsistensi simpul u.
        """
        if u != self.goal_node:
            # rhs(u) = min_{s' in Succ(u)} (c(u, s') + g(s'))
            min_cost = float('inf')
            for neighbor in self.graph.get_neighbors(u):
                weight = self.graph.get_edge_weight(u, neighbor)
                if weight is not None:
                    min_cost = min(min_cost, weight + self.g[neighbor])
            self.rhs[u] = min_cost

        # Hapus u dari antrean jika sudah ada
        self._remove_queue(u)

        # Jika simpul inconsistent (g != rhs), masukkan kembali ke antrean
        if self.g[u] != self.rhs[u]:
            self._insert_queue(u, self._calculate_key(u))

    def compute_shortest_path(self):
        """
        Menghitung ulang/merevisi rute secara bertahap (incremental).
        """
        while self.U and (self.U[0][0] < self._calculate_key(self.start_node) or 
                          self.rhs[self.start_node] != self.g[self.start_node]):
            
            k_old, u = heapq.heappop(self.U)
            self.open_set.discard(u)

            k_new = self._calculate_key(u)

            if k_old < k_new:
                self._insert_queue(u, k_new)
            elif self.g[u] > self.rhs[u]: # Overconsistent
                self.g[u] = self.rhs[u]
                # Perbarui semua tetangga pendahulu (predecessors)
                for predecessor in self.graph.get_predecessors(u):
                    self._update_vertex(predecessor)
            else: # Underconsistent
                self.g[u] = float('inf')
                for predecessor in self.graph.get_predecessors(u) + [u]:
                    self._update_vertex(predecessor)

    # =========================================================================
    # EKSEKUSI RUTE & EVENT DINAMIS (INTEGRASI TEMPLATE)
    # =========================================================================

    def find_path(self):
        """
        Menghasilkan lintasan terpendek berdasarkan nilai g saat ini.
        """
        self.compute_shortest_path()

        if self.g[self.start_node] == float('inf'):
            return [], float('inf')

        # Rekonstruksi rute dari start ke goal
        path = [self.start_node]
        current = self.start_node

        while current != self.goal_node:
            best_neighbor = None
            best_cost = float('inf')

            for neighbor in self.graph.get_neighbors(current):
                weight = self.graph.get_edge_weight(current, neighbor)
                if weight is not None:
                    cost = weight + self.g[neighbor]
                    if cost < best_cost:
                        best_cost = cost
                        best_neighbor = neighbor

            if best_neighbor is None or best_cost == float('inf'):
                return [], float('inf')

            current = best_neighbor
            path.append(current)

        return path, self.g[self.start_node]

    def handle_dynamic_event(self, changed_edges):
        """
        Dipanggil saat ada perubahan bobot edge akibat kejadian dinamis.
        """
        for u, v, old_weight, new_weight in changed_edges:
            # Perbarui bobot di graf
            self.graph.update_edge_weight(u, v, new_weight)
            
            # Perbarui simpul terdampak
            self._update_vertex(u)

        # Hitung ulang rute secara bertahap
        self.compute_shortest_path()

# =========================================================================
# REGISTRASI ALGORITMA KE TEMPLATE
# =========================================================================

AlgorithmRegistry.register("dstar_lite", DStarLite)