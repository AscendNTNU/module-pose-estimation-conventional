//
// Created by marius on 04.11.2020.
//

#ifndef MODULE_POSE_ESTIMATION_CONVENTIONAL_LOOKUPQUEUE_H
#define MODULE_POSE_ESTIMATION_CONVENTIONAL_LOOKUPQUEUE_H

#include <vector>
#include <cstdio>
#include <stdexcept>

template <class ID, class T>
class LookupQueue {
private:
    std::vector<std::pair<ID, T>> container;
    int start{0}, end{0};  // Start inclusive, end not inclusive
    unsigned int max_size;
    void remove_front();
public:
    explicit LookupQueue(unsigned int max_size);

    void push_elem(const ID& id, const T& elem);
    [[nodiscard]] int buffer_size() const {return start <= end ? end - start : end - start + max_size;}
    bool lookupById(const ID& id, T& ret_elem) const;
};

template<class ID, class T>
void LookupQueue<ID, T>::push_elem(const ID &id, const T &elem) {

    if (this->buffer_size() == max_size) {
        remove_front();
    }

    if (end >= max_size)
        end = 0;

    container.at(end) = std::pair<ID, T>(id, elem);

    end += 1;
}

template<class ID, class T>
LookupQueue<ID, T>::LookupQueue(unsigned int max_size) : max_size(max_size) {
    container = std::vector<std::pair<ID, T>>(this->max_size);
};

template<class ID, class T>
void LookupQueue<ID, T>::remove_front() {
    if (buffer_size() < 1)
        throw std::logic_error("Tried to remove element from empty queue. Check max_size > 0.");
    start += 1;
    if (start == max_size)
        start = 0;
}


template<class ID, class T>
bool LookupQueue<ID, T>::lookupById(const ID &id, T& ret_elem) const {
    for (const std::pair<ID, T>& elem: container) {
        if (elem.first == id) {
            ret_elem = elem.second;
            return true;
        }
    }
    return false;
}

#endif //MODULE_POSE_ESTIMATION_CONVENTIONAL_LOOKUPQUEUE_H
