//Elaborando una calculadora

// Clase principal //Mayus primera letra (Prime) modo correcto
public class prime {
  int x;
  int y; // atributos
  float r;

  // Metodo con el mismo nombre de la clase principal es el Constructor.
  public prime(int a, int b, int c) {
    x = a;
    y = b;
    r = c;
  }

  // Otros metodos
  public float suma(int a, int b) {
    x = a;
    y = b;
    r = x + y;
    return r;
  }

  public float resta(int a, int b) {
    x = a;
    y = b;
    r = x - y;
    return r;
  }

  public float multi(int a, int b) {
    x = a;
    y = b;
    r = x * y;
    return r;
  }

  public float div(int a, int b) {
    x = a;
    y = b;
    r = (float) x / (float) y;
    return r;
  }

  public void impResultado(String operacion) { // creo el atributo "operacion" en cual ingreso en System.out... 
  // pero cuando haga el llamado de la clase impResultado tengo que declarar algo para
    // llenar el espacio de "operacion"
    System.out.println("El resultado de la " + " " + operacion + " " + " es =" + " " + r);
  }

  public static void main(String[] args) {
    prime objeto1 = new prime(0, 0, 0); // Inicializa con valores 0

    System.out.println("Valores iniciales" + " " + objeto1.x + " " + objeto1.y + " " + objeto1.r);

    objeto1.suma(2, 8);
    objeto1.impResultado("suma"); // si no agrego "suma" no funciona el programa

    objeto1.resta(10, 4);
    objeto1.impResultado("resta");

    objeto1.multi(3, 6);
    objeto1.impResultado("multiplicacion");

    objeto1.div(10, 4);
    objeto1.impResultado("division");

  }

}