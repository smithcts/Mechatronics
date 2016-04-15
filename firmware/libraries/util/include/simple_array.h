#ifndef SIMPLE_ARRAY_H_INCLUDED
#define SIMPLE_ARRAY_H_INCLUDED

// Type of array indices passed/returned from this class. Signed so can represent invalid indices.
typedef int32_t array_idx_t;

#define INVALID_ARRAY_INDEX (-1)

// Stores elements of the specified type 'T' in a fixed size array.
template <class T>
class SimpleArray
{
  public: // methods

    // Constructor. Dynamically allocates buffer with the specified number of elements.
    SimpleArray(uint32_t max_num_elements) :
          max_num_elements_(max_num_elements)
    {
        data_ = new T[max_num_elements_];
        valid_ = new bool[max_num_elements_];

        for (uint32_t i = 0; i < max_num_elements_; ++i)
        {
            valid_[i] = false;
        }
    }

    // Destructor.
    ~SimpleArray(void)
    {
        delete[] data_;
        delete[] valid_;
    }

    // Copy 'data' into the array.  If successful then return an index that can be used with other methods
    // for accessing/deleting.  If data can't be copied into array (e.g. no room) then return INVALID_ARRAY_INDEX.
    array_idx_t add(T & data);

    // Return the memory address of an element in the array that hasn't been filled with valid data yet. Useful for
    // custom implementations of 'copying' data into the array.  If successful then will return memory address of
    // open element and store it's associated element idx in the output parameter.  If no available elements then
    // return null and idx will be set to an invalid index.
    void * requestStorage(array_idx_t * idx);

    // Mark the data associated with specified 'idx' as being invalid so it can be used for new data in the future.
    // Return true if successful.
    bool remove(array_idx_t idx);

    // Return the memory address of that data associated with 'idx'. If there is no valid data associated with idx
    // then return null.
    void * reference(array_idx_t idx);

    // Copy the data associated with 'idx' into the output parameter 'element'.  If there's not valid data associated
    // with idx then return false.
    bool get(array_idx_t idx, T * element);

  private: // fields

      T * data_;      // Backing array (buffer) for the actual data.
      bool * valid_;  // Array of flags that are true if the corresponding index in 'data_' is valid.

      uint32_t max_num_elements_; // Maximum number of elements that array can hold.

};

//*****************************************************************************
template <class T>
array_idx_t SimpleArray<T>::add(T & new_data)
{
    array_idx_t first_open_index = INVALID_ARRAY_INDEX;

    for (array_idx_t i = 0; i < max_num_elements_; ++i)
    {
        if (!valid_[i])
        {
            first_open_index = i;
            data_[i] = new_data;
            valid_[i] = true;
            break;
        }
    }

    return first_open_index;
}

//*****************************************************************************
template <class T>
void * SimpleArray<T>::requestStorage(array_idx_t * idx)
{
    *idx = INVALID_ARRAY_INDEX;

    for (array_idx_t i = 0; i < max_num_elements_; ++i)
    {
        if (!valid_[i])
        {
            *idx = i;
            valid_[i] = true;
            return data_ + i;
        }
    }

    return NULL; // no open spots
}

//*****************************************************************************
template <class T>
bool SimpleArray<T>::remove(array_idx_t idx)
{
    if ((idx < 0) || (idx >= max_num_elements_))
    {
        return false;
    }

    valid_[idx] = false;
    return true; // success
}

//*****************************************************************************
template <class T>
void * SimpleArray<T>::reference(array_idx_t idx)
{
    if ((idx < 0) || (idx >= max_num_elements_))
    {
        return NULL;
    }

    return data_ + idx;
}

#endif

