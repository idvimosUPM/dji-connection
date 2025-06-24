package org.tfg.custom.types;

import java.util.Objects;

public class Value {
    public static final Value VOID = new Value(new Object()); // Representa una sentencia que no devuelve valor

    private final Object value; // Puede ser Double, Boolean, String, Integer, or null

    public Value(Object value) {
        this.value = value;
    }

    public boolean isDouble() {
        return value instanceof Double || value instanceof Integer;
    }

    public boolean isBoolean() {
        return value instanceof Boolean;
    }

    public boolean isString() {
        return value instanceof String;
    }

    public Double asDouble() {
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        }
        return (Double) value;
    }

    public Boolean asBoolean() {
        return (Boolean) value;
    }

    public String asString() {
        return String.valueOf(value);
    }

    public Object getActualValue() {
        return value;
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(value);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Value other = (Value) obj;
        return Objects.equals(this.value, other.value);
    }

    @Override
    public String toString() {
        return value != null ? value.toString() : "nil";
    }
}