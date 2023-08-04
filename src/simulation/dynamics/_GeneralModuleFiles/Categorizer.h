/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CATEGORIZER_H
#define CATEGORIZER_H

#include "simulation/dynamics/KinematicsArchitecture//KinematicsEngine.h"

class Categorizer {
public:
    explicit Categorizer(int chainLength) : maxChainLength(chainLength) {};
    ~Categorizer() = default;

    int maxChainLength;

    std::shared_ptr<Node> findBaseNode(const std::vector<std::shared_ptr<Node>>& nodeList);  // use std::optional here?
private:
    bool checkBase(const std::shared_ptr<Node>& node);
    double search(const std::shared_ptr<Node>& node, const std::shared_ptr<Node>& parentNode);
    void unvisitNodes(const std::vector<std::shared_ptr<Node>>& nodeList);

};

#endif
