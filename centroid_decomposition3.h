#include <iostream>
#include <unordered_set>
#include <vector>
#include <cassert>
#include <algorithm>
#include <deque>

using tree_edges = std::vector<std::pair<int, int>>;

using tree_nodes = std::vector<std::vector<int>>;

struct tree_node
{
  tree_node(int d)
    : data_( d )
  {

  }
  int data_;
  int depth_ = -1;
  std::unordered_set<tree_node*> siblings_;

  tree_node* first_sibling() const
  {
    //assert(siblings_.size());
    if (siblings_.empty()) {
      return nullptr;
    }
    auto ret = *siblings_.begin();
    return ret;
  }
  tree_node* next_sibling(tree_node* sibling) const
  {
    auto iter = siblings_.find(sibling);
    // sanity check
    assert(iter != siblings_.end());

    ++iter;

    if (iter == siblings_.end()) {
      return nullptr;
    }
    return *iter;
  }
  void remove_itself_from_siblings()
  {
    for (auto other : siblings_) {
      other->siblings_.erase(this);
    }
  }
};

struct tree_op
{
  std::vector<tree_node*> tree_nodes_; //owner, indexed by number
  tree_op(const tree_edges& edges)
  {
    fill_tree_nodes(edges);
  }
  ~tree_op()
  {
    clear();
  }
  void clear()
  {
    // TODO: make exception safe
    for (auto n : tree_nodes_) {
      delete n;
    }
  }

  void fill_tree_nodes(const tree_edges& edges)
  {
    // amount of nodes in a tree == amount_of_edges + 1
    tree_nodes_.resize(edges.size() + 1, nullptr);
    for (const auto& edge : edges) {
      tree_node *node1 = nullptr, *node2 = nullptr;

      if (    tree_nodes_.at(edge.first - 1) == nullptr
           && tree_nodes_.at(edge.second - 1) == nullptr) {
        node1 = new tree_node(edge.first);
        node2 = new tree_node(edge.second);
        tree_nodes_.at(node1->data_ - 1) = node1;
        tree_nodes_.at(node2->data_ - 1) = node2;
      }
      else if (tree_nodes_.at(edge.first - 1) == nullptr) {
        node1 = new tree_node(edge.first);
        tree_nodes_.at(node1->data_ - 1) = node1;
        node2 = tree_nodes_.at(edge.second - 1);
      }
      else if (tree_nodes_.at(edge.second - 1) == nullptr) {
        node2 = new tree_node(edge.second);
        tree_nodes_.at(node2->data_ - 1) = node2;
        node1 = tree_nodes_.at(edge.first - 1);
      }
      else {
        //sanity check, both nodes cannot exist
        assert(false);
      }

      node1->siblings_.insert(node2);
      node2->siblings_.insert(node1);
    }
  }

  std::vector<int> count_subtree_sizes_dfs(int start_node) const
  {
    std::vector<int> visited(tree_nodes_.size());

    std::vector<std::pair<tree_node*, tree_node*>> node_stack;
    {
      auto node_to_visit = tree_nodes_.at(start_node);

      node_stack.push_back(std::make_pair(node_to_visit, node_to_visit->first_sibling()));
      visited.at(node_to_visit->data_ - 1) = 1;
    }

    std::vector<int> ret(tree_nodes_.size());
    for (;;) {
      if (node_stack.empty()) {
        break;
      }

      auto& cur_node_info = node_stack.back();

      {
        auto cur_node = cur_node_info.first;
        auto cur_sibling = cur_node_info.second;
        if (cur_sibling == nullptr) {
          // no sibling for the cur node push it out

          node_stack.pop_back();
          if (node_stack.empty() == false) {
            auto parent_node = node_stack.back().first;
            ret.at(parent_node->data_ - 1) += 1 + ret.at(cur_node->data_ - 1);
          }
          continue;
        }
        if (visited.at(cur_sibling->data_ - 1) == 1) {
          // node is visited already, prepare next one and iterate again
          cur_node_info.second = cur_node->next_sibling(cur_sibling);
          continue;
        }

        visited.at(cur_sibling->data_ - 1) = 1;
        node_stack.push_back(std::make_pair(cur_sibling, cur_sibling->first_sibling()));
      }
    }
    return ret;
  }

  int find_centroid(int node_idx, const std::vector<int>& subtree_depth) const
  {
    const int tree_size = tree_nodes_.size();

    auto node = tree_nodes_.at(node_idx);
    std::vector<int> visited(tree_size);
    visited.at(node->data_ - 1) = 1;
    for (;;) {

      for (auto
             sibling = node->first_sibling()
           ; sibling != nullptr
           ; sibling = node->next_sibling(sibling)) {
        if (visited.at(sibling->data_ - 1) == 1) {
          continue;
        }
        visited.at(sibling->data_ - 1) = 1;
        auto child_size = subtree_depth.at(sibling->data_ - 1);
        if ((child_size + 1) > tree_size / 2) {
          node = sibling;
          break;
        }
      }
      // if we passed the loop then the node is a centroid
      return node->data_;
    }
  }

  int centroid_decompose(int node_num, tree_edges& centroid_edges) const
  {
    auto subtree_depth = count_subtree_sizes_dfs(node_num);
    int centroid_num = find_centroid(node_num, subtree_depth);
    auto centroid_node = tree_nodes_.at(centroid_num - 1);
    centroid_node->remove_itself_from_siblings();
    for (auto sibling : centroid_node->siblings_) {
      int edge_end = centroid_decompose(sibling->data_ - 1, centroid_edges);
      int edge_start = centroid_node->data_ - 1;
      centroid_edges.push_back(std::make_pair(edge_start + 1, edge_end + 1));
    }
    return centroid_node->data_ - 1;
  }

  void mark_depths_dfs(int root_node)
  {
    auto n = tree_nodes_.at(root_node);
    n->depth_ = 0;

    std::deque<tree_node*> node_queue;
    node_queue.push_front(n);

    std::vector<int> visited_nodes(tree_nodes_.size());
    visited_nodes.at(n->data_- 1) = 1;

    for (;;) {
      if (node_queue.empty()) {
        break;
      }

      auto n = node_queue.back();
      node_queue.pop_back();


      for (auto
             sibling = n->first_sibling()
           ; sibling
           ; sibling = n->next_sibling(sibling)) {

        if (visited_nodes.at(sibling->data_ - 1) == 1) {
          continue;
        }
        visited_nodes.at(sibling->data_ - 1) = 1;

        node_queue.push_front(sibling);

        sibling->depth_ = n->depth_ + 1;
      }

    }
  }
};

// only keep edges to parent
struct reversed_tree
{
  reversed_tree(const tree_edges& edges)
  {
    construct(edges);
  }
  void construct(const tree_edges& edges)
  {
    nodes_.resize(edges.size() + 1, -1);
    for (const auto& e : edges) {
      nodes_.at(e.second - 1) = e.first - 1;
    }
  }
  std::vector<int> nodes_;
};

