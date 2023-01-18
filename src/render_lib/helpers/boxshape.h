#pragma once

template <typename B>
class BoxShape {
    public:
    BoxShape(B width, B height, B depth) : width{width}, height{height}, depth{depth} {};
    B width;
    B height;
    B depth;

    B size() {
        return width * height * depth;
    };
};