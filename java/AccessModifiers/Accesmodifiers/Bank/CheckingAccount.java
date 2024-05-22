public class CheckingAccount{

    private String name;
    private int balance;
    private String id;
    private double interestRate;


    public CheckingAccount (String inputName, int inputBalance, String inputId){

        this.name = inputName;                   //this? Establecer diferencia entre el atributo propio
        this.balance = inputBalance;             // de mi clase y lo que esta afuera.
        this.id = inputId;
        this.interestRate = 0.02;
    }

         //Metodos
         
    public int getBalance(){  
        return this.balance;
    }

    public void setBalance(int newBalance){
        this.balance = newBalance;
    }

    public double getMonthlyInterest(){
        return this.interestRate * this.balance;
    }

    public void consignar(int valorConsignado){
        balance= balance + valorConsignado;
        
    }

    public void retirar(int valorRetirado){
        balance= balance - valorRetirado;
    }
}