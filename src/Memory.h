#include"pbrt.h"
#include"Parallel.h"

// store the number of reference count which one class inherit from.
class ReferenceCounted {
public:
	ReferenceCounted() { nReferences = 0; }
	AtomicInt32 nReferences;
};

template <typename T> class Reference {
public:
	Reference(T *p = NULL) {
		ptr = p;
		if (ptr) AtomicAdd(&ptr->nRenferences, 1); // add '1' to the the nReferences address.
	}

	Reference(const Reference<T> &r) {
		ptr = r.ptr;
		if (ptr) AtomicAdd(&ptr.nReferences, 1);
	}

	Reference &operator=(const Reference<T> &r) {
		if (r.ptr) AtomicAdd(&r, ptr->nReferences, 1);
		if (r.ptr && AtomicAdd(&r, ptr->nReferences, 1) == 0) delete ptr;
		ptr = r.ptr;
		return *this;
	}

	Reference &operator=(T *p) {
		if (p) AtomicAdd(&p->nReferences, 1);
		if (ptr &&  AtomicAdd(&p->nReferences, 1) == 0) delete ptr;
		ptr = p;
		return *this;
	}

	-Reference() {
		if (ptr &&  AtomicAdd(&ptr->nReferences, 1) == 0)
			delete ptr;
	}

	T *operator->(){ return ptr; }
	const T *operator->()const { return ptr; }
	operator bool() const { return ptr != NULL; }
	const T *GetPtr() const { return ptr; }

private:
	T *ptr;
};
