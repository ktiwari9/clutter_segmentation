/*
   ARPACK++ v1.2 2/20/2000
   c++ interface to ARPACK code.

   MODULE ARGSym.h.
   Arpack++ class ARSymGenEig definition.

   ARPACK Authors
      Richard Lehoucq
      Danny Sorensen
      Chao Yang
      Dept. of Computational & Applied Mathematics
      Rice University
      Houston, Texas
*/

#ifndef ARGSYM_H
#define ARGSYM_H

#include <stddef.h>
#include "arch.h"
#include "arssym.h"
#include "arrgsym.h"
#include "argeig.h"

template<class ARFLOAT, class ARFOP, class ARFB>
class ARSymGenEig:
  virtual public ARGenEig<ARFLOAT, ARFLOAT, ARFOP, ARFB>,
  virtual public ARSymStdEig<ARFLOAT, ARFOP>,
  virtual public ARrcSymGenEig<ARFLOAT> {

	  typedef ARGenEig<ARFLOAT, ARFLOAT, ARFOP, ARFB> genSuperT;

 public:

 // a) Notation.

  typedef void (ARFB::* TypeBx)(ARFLOAT[], ARFLOAT[]);


 protected:

 // b) Protected variables:

  ARFB    *objA;      // Object that has MultAx as a member function.
  TypeBx  MultAx;     // Function that evaluates the product A*x.

 // c) Protected functions:

  virtual void Copy(const ARSymGenEig& other);
  // Makes a deep copy of "other" over "this" object.
  // Old values are not deleted (this function is to be used
  // by the copy constructor and the assignment operator only).


 public:

 // d) Public functions:

 // d.1) Functions that allow changes in problem parameters.

  void SetShiftInvertMode(ARFLOAT sigmap, ARFOP* objOPp,
                          void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]));
  // Turns problem to shift and invert mode with shift defined by sigmap.

  void SetBucklingMode(ARFLOAT sigmap, ARFOP* objOPp,
                       void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]));
  // Turns problem to buckling mode with shift defined by sigmap.

  void SetCayleyMode(ARFLOAT sigmap, ARFOP* objOPp,
                     void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]),
                     ARFB* objAp, void (ARFB::* MultAxp)(ARFLOAT[], ARFLOAT[]));
  // Turns problem to Cayley mode with shift defined by sigmap.


 // d.2) Functions that perform all calculations in one step.

  int FindArnoldiBasis();
  // Determines the Arnoldi basis related to the given problem.


 // d.3) Constructors and destructor.

  ARSymGenEig() { genSuperT::InvertMode = 'S'; }
  // Short constructor that does almost nothing.

  ARSymGenEig(int np, int nevp, ARFOP* objOPp,
              void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]), ARFB* objBp,
              void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]),
              const char* whichp = "LM", int ncvp = 0, ARFLOAT tolp = 0.0,
              int maxitp = 0, ARFLOAT* residp = NULL, bool ishiftp = true);
  // Long constructor (regular mode).

  ARSymGenEig(char invertmodep, int np, int nevp, ARFOP* objOPp,
              void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]),
              ARFB* objBp, void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]),
              ARFLOAT sigmap, const char* whichp = "LM", int ncvp = 0,
              ARFLOAT tolp = 0.0, int maxitp = 0, ARFLOAT* residp = NULL,
              bool ishiftp = true);
  // Long constructor (shift-and-invert and buckling mode).

  ARSymGenEig(int np, int nevp, ARFOP* objOPp,
              void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]), ARFB* objAp,
              void (ARFB::* MultAxp)(ARFLOAT[], ARFLOAT[]), ARFB* objBp,
              void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]), ARFLOAT sigmap,
              const char* whichp = "LM", int ncvp = 0, ARFLOAT tolp = 0.0,
              int maxitp = 0, ARFLOAT* residp = NULL, bool ishiftp = true);
  // Long constructor (cayley mode).

  ARSymGenEig(const ARSymGenEig& other) { Copy(other); }
  // Copy constructor.

  virtual ~ARSymGenEig() { }
  // Destructor.

 // e) Operators.

  ARSymGenEig& operator=(const ARSymGenEig& other);
  // Assignment operator.

}; // class ARSymGenEig.


// ------------------------------------------------------------------------ //
// ARSymGenEig member functions definition.                                 //
// ------------------------------------------------------------------------ //


template<class ARFLOAT, class ARFOP, class ARFB>
inline void ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
Copy(const ARSymGenEig<ARFLOAT, ARFOP, ARFB>& other)
{

  ARGenEig<ARFLOAT, ARFLOAT, ARFOP, ARFB>::Copy(other);
  objA       = other.objA;
  MultAx     = other.MultAx;
  genSuperT::InvertMode = other.InvertMode;

} // Copy.


template<class ARFLOAT, class ARFOP, class ARFB>
void ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
SetShiftInvertMode(ARFLOAT sigmap, ARFOP* objOPp,
                   void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]))
{

	genSuperT::InvertMode = 'S';
  genSuperT::objOP      = objOPp;
  genSuperT::MultOPx    = MultOPxp;
  ChangeShift(sigmap);

} // SetShiftInvertMode.


template<class ARFLOAT, class ARFOP, class ARFB>
void ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
SetBucklingMode(ARFLOAT sigmap, ARFOP* objOPp,
                void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]))

{

	genSuperT::InvertMode = 'B';
	genSuperT::objOP      = objOPp;
	genSuperT::MultOPx    = MultOPxp;
  ChangeShift(sigmap);

} // SetBucklingMode.


template<class ARFLOAT, class ARFOP, class ARFB>
void ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
SetCayleyMode(ARFLOAT sigmap, ARFOP* objOPp,
              void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]), ARFB* objAp,
              void (ARFB::* MultAxp)(ARFLOAT[], ARFLOAT[]))

{

	genSuperT::InvertMode = 'C';
	genSuperT::objOP      = objOPp;
	genSuperT::MultOPx    = MultOPxp;
  objA       = objAp;
  MultAx     = MultAxp;
  ChangeShift(sigmap);

} // SetCayleyMode.


template<class ARFLOAT, class ARFOP, class ARFB>
int ARSymGenEig<ARFLOAT, ARFOP, ARFB>::FindArnoldiBasis()
{

  ARFLOAT* temp;

  if (genSuperT::mode != 5) {  // Using base function if not in Cayley mode.
    return ARGenEig<ARFLOAT, ARFLOAT, ARFOP, ARFB>::FindArnoldiBasis();
  }
  else {

    temp = new ARFLOAT[genSuperT::n+1];

    if (!genSuperT::BasisOK) genSuperT::Restart();

    // Changing to auto shift mode.

    if (!genSuperT::AutoShift) {
      ArpackError::Set(ArpackError::CHANGING_AUTOSHIFT, "FindArnoldiBasis");
      genSuperT::AutoShift=true;
    }

    // ARPACK main loop.

    while (!genSuperT::BasisOK) {

      // Calling Aupp.

      try { genSuperT::TakeStep(); }
      catch (ArpackError) {
        ArpackError(ArpackError::CANNOT_FIND_BASIS, "FindArnoldiBasis");
        delete[] temp;
        return 0;
      }

      switch (genSuperT::ido) {
      case -1:

        // Performing y <- B*x for the first time.

      genSuperT::ipntr[3] = genSuperT::ipntr[2]+genSuperT::n; // not a clever idea, but...
        (genSuperT::objB->*genSuperT::MultBx)(&genSuperT::workd[genSuperT::ipntr[1]],&genSuperT::workd[genSuperT::ipntr[3]]);

      case  1:

        // Performing y <- OP*(A+sigma*B)*x, B*x is already available.

        (genSuperT::objB->*MultAx)(&genSuperT::workd[genSuperT::ipntr[1]], temp);
        axpy(genSuperT::n, genSuperT::sigmaR, &genSuperT::workd[genSuperT::ipntr[3]], 1, temp, 1);
        (genSuperT::objOP->*genSuperT::MultOPx)(temp, &genSuperT::workd[genSuperT::ipntr[2]]);
        break;

      case  2:

        // Performing y <- B*x.

        (genSuperT::objB->*genSuperT::MultBx)(&genSuperT::workd[genSuperT::ipntr[1]],&genSuperT::workd[genSuperT::ipntr[2]]);

      }
    }

    delete[] temp;

    return genSuperT::nconv;
  }

} // FindArnoldiBasis.


template<class ARFLOAT, class ARFOP, class ARFB>
inline ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
ARSymGenEig(int np, int nevp, ARFOP* objOPp,
            void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]),
            ARFB* objBp, void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]),
            const char* whichp, int ncvp, ARFLOAT tolp, int maxitp,
            ARFLOAT* residp, bool ishiftp)

{

	genSuperT::InvertMode = 'S';
  genSuperT::NoShift();
  DefineParameters(np, nevp, objOPp, MultOPxp, objBp, MultBxp,
                   whichp, ncvp, tolp, maxitp, residp, ishiftp);

} // Long constructor (regular mode).


template<class ARFLOAT, class ARFOP, class ARFB>
inline ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
ARSymGenEig(char InvertModep, int np, int nevp, ARFOP* objOPp,
            void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]),
            ARFB* objBp, void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]),
            ARFLOAT sigmap, const char* whichp, int ncvp, ARFLOAT tolp,
            int maxitp, ARFLOAT* residp, bool ishiftp)

{

	genSuperT::InvertMode = genSuperT::CheckInvertMode(InvertModep); // InvertMode = 'S' or 'B'.
  ChangeShift(sigmap);
  DefineParameters(np, nevp, objOPp, MultOPxp, objBp, MultBxp,
                   whichp, ncvp, tolp, maxitp, residp, ishiftp);

} // Long constructor (shift-and-invert and buckling mode).


template<class ARFLOAT, class ARFOP, class ARFB>
inline ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
ARSymGenEig(int np, int nevp, ARFOP* objOPp,
            void (ARFOP::* MultOPxp)(ARFLOAT[], ARFLOAT[]),
            ARFB* objAp, void (ARFB::* MultAxp)(ARFLOAT[], ARFLOAT[]),
            ARFB* objBp, void (ARFB::* MultBxp)(ARFLOAT[], ARFLOAT[]),
            ARFLOAT sigmap, const char* whichp, int ncvp, ARFLOAT tolp,
            int maxitp, ARFLOAT* residp, bool ishiftp)

{

  SetCayleyMode(sigmap, objOPp, genSuperT::MultOPx, objAp, MultAxp);
  DefineParameters(np, nevp, objOPp, MultOPxp, objBp, MultBxp,
                   whichp, ncvp, tolp, maxitp, residp, ishiftp);

} // Long constructor (cayley mode).


template<class ARFLOAT, class ARFOP, class ARFB>
ARSymGenEig<ARFLOAT, ARFOP, ARFB>& ARSymGenEig<ARFLOAT, ARFOP, ARFB>::
operator=(const ARSymGenEig<ARFLOAT, ARFOP, ARFB>& other)
{

  if (this != &other) { // Stroustrup suggestion.
	  genSuperT::ClearMem();
    Copy(other);
  }
  return *this;

} // operator=.


#endif // ARGSYM_H

