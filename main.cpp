#include "centroid_decomposition3.h"
// kitties calculation on a tree

tree_edges fill_test_data()
{
  tree_edges ret =
  {
     {1,2}
   , {1,3}
   , {1,4}
   , {3,5}
   , {3,6}
   , {3,7}
  };
  return ret;
}


int main()
{
  tree_edges edges = fill_test_data();
  std::vector<int> seq = { 2, 4, 5 };
  std::vector<int> seq_index(100000, 0);
  for (auto q : seq) {
    seq_index.at(q - 1) = 1;
  }

  tree_edges centroid_edges;
  tree_op tree(edges);
  int centroid_root = -1;
  {
    tree_op tree_for_centroid(edges);
    centroid_root = tree_for_centroid.centroid_decompose(0, centroid_edges);
  }

  tree.mark_depths_bfs(0);

  std::reverse(centroid_edges.begin(), centroid_edges.end());

  tree_op centroid_tree(centroid_edges);
  centroid_tree.count_subtree_sum_dfs(centroid_root, seq_index);
  centroid_tree.mark_parent_nodes_dfs(centroid_root);
  //reversed_tree centroid_tree(centroid_edges);

  for (auto q : seq) {
    auto cur_node = centroid_tree.tree_nodes_.at(q - 1);
    auto parent_node_lca = cur_node->parent_node_;
    for (;;) {
      if (parent_node_lca == nullptr) {
        break;
      }
      // distance between currrent node and LCA
      tree.
    }
  }
  return 0;
}

