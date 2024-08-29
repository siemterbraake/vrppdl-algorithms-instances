/*
The capacitated sorted deque struct is a data structure that stores a sorted deque with a maximum capacity. 
It is used in the ALNS algorithm to store the best solutions and labels found so far.

Written by: Siem ter Braake, 2024
*/

#ifndef CAPICATEDSORTEDDEQUE_H
#define CAPICATEDSORTEDDEQUE_H

#include<deque>
#include<iomanip>
#include<set>


template<typename T>
struct CapacitatedSortedDeque {
    std::deque<T> d_deque;
    unsigned short d_capacity;
    std::set<std::size_t> d_hashes;

    explicit CapacitatedSortedDeque() {};
    explicit CapacitatedSortedDeque(unsigned short capacity);
    void insert(const T& value, std::size_t hash);
    bool isFilled();
    void clear();
    T min() const;
    T max() const;
    typename std::deque<T>::iterator begin() {
        return d_deque.begin();
    }
    typename std::deque<T>::iterator end() {
        return d_deque.end();
    }
    operator std::deque<T>() const {
        return d_deque;
    }
};

template<typename T>
CapacitatedSortedDeque<T>::CapacitatedSortedDeque(unsigned short capacity) : d_capacity(capacity) {};

template<typename T>
void CapacitatedSortedDeque<T>::insert(const T& value, std::size_t hash) {
    // If the hash is already in the set, do not insert the value
    if (d_hashes.find(hash) != d_hashes.end()) {
        return;
    }
    d_hashes.insert(hash);
    // If capacity is reached
    if (d_deque.size() >= d_capacity) {
        // Check if the new value is less than the highest value in the deque
        if (value < d_deque.back()) {
            // Remove the highest value to make room for the new value
            d_deque.pop_back();
        } else {
            // If the new value is not less, do not insert it
            return;
        }
    }

    // Find the correct insertion point to keep the deque sorted
    auto it = std::lower_bound(d_deque.begin(), d_deque.end(), value);
    d_deque.insert(it, value);
}

template<typename T>
void CapacitatedSortedDeque<T>::clear() {
    d_deque.clear();
    d_hashes.clear();
}

template<typename T>
bool CapacitatedSortedDeque<T>::isFilled() {
    return d_deque.size() == d_capacity;
}

template<typename T>
T CapacitatedSortedDeque<T>::min() const {
    if (d_deque.empty()) {
        throw std::runtime_error("Deque is empty");
    }
    return d_deque.front();
}

template<typename T>
T CapacitatedSortedDeque<T>::max() const {
    if (d_deque.empty()) {
        throw std::runtime_error("Deque is empty");
    }
    return d_deque.back();
}

#endif