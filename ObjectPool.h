#ifndef _ObjectPool_h_
#define _ObjectPool_h_

#include <DIAG.h>

#define MAXPOOLSIZE 32

template <typename T, int length>
class ObjectPool
{

    // just make sure that we don't create a pool eating up all memory @compiletime
    static_assert(length <= MAXPOOLSIZE); 
    struct item
    {
        T i;
        bool free = true; // boolean 1 free i.e. i can be reused; 0 occupied
    };

private:
    item p[length];          // MAXPOOLSIZE items of struct item
    const int size = length; // size of the pool

    int findFreeIdx()
    { // find the first free index or return -1 if there is none
        for (int i = 0; i < length; i++)
        {
            if (p[i].free)
            {
                return i;
            }
        }
        return -1; // if we are here there is no free slot available
    }

public:
    int setItem(T i)
    { // add an item to the pool at a free slot
        int idx = findFreeIdx();
        if (idx != -1)
        {
            p[idx].i = i;
            p[idx].free = false;
        }
        return idx;
    }

    /**
     * @brief returns the slot for an object to the pool i.e. frees the slot for reuse of the data member and
     * clears out the memory
     * 
     * @param idx 
     * @return true  if the return is ok
     * @return false otherwise
     */
    bool returnItem(int idx)
    { // clear item at pool index idx
        if (idx > size)
        { // can't return an item outside of the pool size; returns false;
            return false;
        }
        memset(&p[idx].i, 0, sizeof(T)); // clear out the memory but keep the allocation for reuse
        p[idx].free = true;
        return true; // set the free flag
    }

    /**
     * @brief Obtain a pool item
     * @note This should only be used for debugging. 
     * It allows to change actually the content of the pool item where this should only be allowed for the setItem method.
     * @param idx Index of the pool item to retrieve 
     * @param state State of the pool item ( 1 available, 0 occupied)
     * @return T* returns the pointer to the pool item
     */
    T *getItem(int idx, bool *state)
    {
        *state = p[idx].free;
        return &p[idx].i;
    }

    int getSize()
    {
        return size;
    }

    ObjectPool() = default;
    ~ObjectPool() = default;
};

#endif