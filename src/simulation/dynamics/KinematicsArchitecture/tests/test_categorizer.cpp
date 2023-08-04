/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "simulation/dynamics/KinematicsArchitecture/Categorizer.h"
#include "simulation/dynamics/KinematicsArchitecture/KinematicsEngine.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/tests/unitTestComparators.h"
#include <gtest/gtest.h>

void connect(const std::shared_ptr<Node>& node1,
             const std::shared_ptr<Joint>& joint,
             const std::shared_ptr<Node>& node2) {
    node1->edgeList.emplace_back(node2, joint);
    node2->edgeList.emplace_back(node1, joint);
}

class Fork : public ::testing::Test {
//           A
//  	    / \
//         1   1
// 	      /     \
// 	     D       B
//      / \       \
//     2   1       1
//    /     \       \
//   G       E       C
//            \
//             1
//              \
//               F
protected:
    int numNodes = 7;
    int numJoints = 6;
    std::vector<std::shared_ptr<Node>> nodeList;
    std::vector<std::shared_ptr<Joint>> jointList;

    void SetUp() override {
        for (int i = 0; i < numNodes; i++) {
            nodeList.push_back(std::make_shared<Node>(std::make_shared<Part>()));
        }

        for (int i = 0; i < numJoints-1; i++) {
            jointList.push_back(std::make_shared<RotaryOneDOF>());
        }
        jointList.push_back(std::make_shared<RotaryTwoDOF>());

        // Connect the nodes
        connect(nodeList[0], jointList[0], nodeList[1]);
        connect(nodeList[1], jointList[1], nodeList[2]);
        connect(nodeList[0], jointList[2], nodeList[3]);
        connect(nodeList[3], jointList[3], nodeList[4]);
        connect(nodeList[4], jointList[4], nodeList[5]);
        connect(nodeList[3], jointList[5], nodeList[6]);
    }
};

TEST_F(Fork, findBaseNode) {
    auto categorizer = Categorizer(2);
    auto node = categorizer.findBaseNode(nodeList);
    EXPECT_TRUE(node.get() == nodeList[3].get());
}

class Chain : public ::testing::TestWithParam<int> {
// A
//  \
//   2
// 	  \
// 	   B
//      \
//       1
//        \
//         C
//          \
//           2
//            \
//             D
protected:
    int numNodes = 5;
    int numJoints = 4;
    std::vector<std::shared_ptr<Node>> nodeList;
    std::vector<std::shared_ptr<Joint>> jointList;

    void SetUp() override {
        for (int i = 0; i < numNodes; i++) {
            nodeList.push_back(std::make_shared<Node>(std::make_shared<Part>()));
        }

        for (int i = 0; i < numJoints; i++) {
            jointList.push_back(std::make_shared<RotaryOneDOF>());
        }

        // Connect the nodes
        connect(nodeList[0], jointList[0], nodeList[1]);
        connect(nodeList[1], jointList[1], nodeList[2]);
        connect(nodeList[2], jointList[2], nodeList[3]);
        connect(nodeList[3], jointList[3], nodeList[4]);
    }
};

TEST_P(Chain, findBaseNode) {
    auto maxChainLength = GetParam();
    auto categorizer = Categorizer(maxChainLength);
    auto node = categorizer.findBaseNode(nodeList);
    if (maxChainLength > 1) {
        EXPECT_TRUE(node.get() == nodeList[4-maxChainLength].get());
    } else {
        EXPECT_TRUE(node.get() == nullptr);  // can't find suitable base node
    }
}

INSTANTIATE_TEST_SUITE_P(
        Categorizer,
        Chain,
        ::testing::Values(
                0, 1, 2, 3, 4
                )
);

class Loop : public ::testing::Test {
// A - 1 - B
// |       |
// 1       1
// |       |
// C - 1 - D
//          \
//           1
//            \
//             E
protected:
    int numNodes = 5;
    int numJoints = 5;
    std::vector<std::shared_ptr<Node>> nodeList;
    std::vector<std::shared_ptr<Joint>> jointList;

    void SetUp() override {
        for (int i = 0; i < numNodes; i++) {
            nodeList.push_back(std::make_shared<Node>(std::make_shared<Part>()));
        }

        for (int i = 0; i < numJoints-1; i++) {
            jointList.push_back(std::make_shared<RotaryOneDOF>());
        }
        jointList.push_back(std::make_shared<RotaryTwoDOF>());

        // Connect the nodes
        connect(nodeList[0], jointList[0], nodeList[1]);
        connect(nodeList[0], jointList[1], nodeList[2]);
        connect(nodeList[1], jointList[2], nodeList[3]);
        connect(nodeList[2], jointList[3], nodeList[3]);
        connect(nodeList[3], jointList[4], nodeList[4]);
    }
};

TEST_F(Loop, findBaseNode)
{
    auto categorizer = Categorizer(2);
    auto node = categorizer.findBaseNode(nodeList);
    EXPECT_TRUE(node.get() == nullptr);
}